/*
 * RobotPoseEKF.cc
 *
 *  Copyright (c) 2015, Norman Alan Oursland
 *  All rights reserved.
 */

#include "cognitoware/math/data/Matrix.h"
#include "cognitoware/math/data/Vector.h"
#include "cognitoware/math/probability/continuous/GaussianMoment.h"
#include "cognitoware/math/probability/RandomDistribution.h"
#include "cognitoware/robotics/state_estimation/ExtendedKalmanActionModel.h"
#include "cognitoware/robotics/state_estimation/ExtendedKalmanSensorModel.h"
#include "gtest/gtest.h"

#include <cmath>
#include <iostream>
#include <math.h>
#include <memory>

using ::cognitoware::math::data::Matrix;
using ::cognitoware::math::data::Vector;
using ::cognitoware::math::probability::continuous::GaussianMoment;
using ::cognitoware::math::probability::RandomDistribution;
using ::cognitoware::robotics::state_estimation::ExtendedKalmanActionModel;
using ::cognitoware::robotics::state_estimation::ExtendedKalmanSensorModel;

namespace examples {
namespace {
class RobotPose : public Vector {
public:
  RobotPose() : Vector({0.0, 0.0, 0.0}) {
  }

  RobotPose(double x, double y, double theta) :
      Vector({x, y, NormalizeTheta(theta)}) {
  }

  void Set(Vector v) override {
    v[2] = NormalizeTheta(v[2]);
    Vector::Set(std::move(v));
  }
private:
  static double NormalizeTheta(double theta) {
    while (theta > M_PI) {
      theta -= M_TWOPI;
    }
    while (theta < -M_PI) {
      theta += M_TWOPI;
    }
    return theta;
  }
};

DEFINE_VECTOR1(SensorReading);
DEFINE_VECTOR1(RobotAction);

class RobotActionModel : public ExtendedKalmanActionModel<RobotAction, RobotPose> {
public:
  RobotActionModel() : r_(3,3) {}

  Matrix GetStateJacobian(const RobotAction& u,
                          const RobotPose& x) const override {
    // g =
    //     d(x+u*Cos(a))/dx      d(x+u*Cos(a))/dy     d(x+u*Cos(a))/da
    // d(range+u*Sin(a))/dx  d(range+u*Sin(a))/dy d(range+u*Sin(a))/da
    //              d(a)/dx               d(a)/dy              d(a)/da
    // =
    // 1  0 -u*Sin(a)
    // 0  1 +u*Cos(a)
    // 0  0 1
    return Matrix(
        3, 3,
        {1, 0, -u[0] * sin(x[2]),
         0, 1, +u[0] * cos(x[2]),
         0, 0, 1});
  }
  Matrix GetActionJacobian(const RobotAction&,
                           const RobotPose& x) const override {
    return Matrix(3, 1, {cos(x[2]), sin(x[2]), 0});
  }
  RobotPose GetMean(const RobotAction& u, const RobotPose& x) const override {
    // g(x,range,a,u) = [(x+u*Cos(a))(range+u*Sin(a))(a)]
    double xNext = x[0] + u[0] * cos(x[2]);
    double yNext = x[1] + u[0] * sin(x[2]);
    double thetaNext = x[2];
    return RobotPose(xNext, yNext, thetaNext);
  }

  Matrix GetError(const RobotAction&, const RobotPose&) const override {
    return r_;
  }

  std::shared_ptr<RandomDistribution<RobotPose>> ConditionBy(
      const RobotAction& u, const RobotPose& x) {
    return std::make_shared<GaussianMoment<RobotPose>>(
        GetMean(u, x),
        GetError(u, x));
  }

  Matrix r_;
};

class RobotSensorModel : public ExtendedKalmanSensorModel<SensorReading, RobotPose> {
public:
  RobotSensorModel() :
      q_(1, 1, {0.01}) {
  }

  double ConditionalProbabilityOf(const SensorReading& observation,
                                  const RobotPose& state) const override {
    if (state[0] == observation[0]) return 1.0;
    return 0.0;
  }

  SensorReading GetMean(const RobotPose& state) const override {
    return SensorReading(state[0]);
  }

  Matrix GetError(const SensorReading&) const override {
    return q_;
  }

  Matrix GetJacobian(const SensorReading&, const RobotPose&) const override {
    // g = d(result)/dx d(result)/dy  d(result)/da
    // = 1  0 0
    return Matrix( 1, 3, {1, 0, 0});
  }

  Matrix q_;
};

std::shared_ptr<GaussianMoment<RobotPose>> DoTimeUpdate(
    const GaussianMoment<RobotPose>& state_belief,
    const RobotAction& u,
    const RobotActionModel& action_model) {
  RobotPose x0 = state_belief.mean();
  Matrix e0 = state_belief.covariance();

  // time update
  Matrix g = action_model.GetStateJacobian(u, x0);
  Matrix gt = g.Transpose();

  RobotPose x1 = action_model.GetMean(u, x0);
  Matrix r = action_model.GetError(u, x0);
  Matrix e1 = g * e0 * gt + r;

  return std::make_shared<GaussianMoment<RobotPose>>(x1, e1);
}

std::shared_ptr<GaussianMoment<RobotPose>> DoMeasurementUpdate(
    const GaussianMoment<RobotPose>& stateBelief,
    const SensorReading& z,
    const Matrix& q,
    const RobotSensorModel& sensorModel) {
  // measurement update
  RobotPose x0 = stateBelief.mean();
  Matrix e0 = stateBelief.covariance();

  Matrix h = sensorModel.GetJacobian(z, x0);
  Matrix ht = h.Transpose();

  Matrix i = Matrix::Identity(e0.order());

  Matrix k = e0 * ht * (h * e0 * ht + q).Inverse();
  Vector v = (x0 + k * (z - sensorModel.GetMean(x0)));
  RobotPose x1(v[0], v[1], v[2]);
  Matrix e1 = (i - k * h) * e0;

  return std::make_shared<GaussianMoment<RobotPose>>(x1, e1);
}
}

TEST(RobotPoseEKF, ActionObsevatonUpdate) {
  int steps = 6;
  std::vector<std::shared_ptr<GaussianMoment<RobotPose>>> beliefs;

  Matrix q(1, 1, {0});

  RobotActionModel action_model;
  RobotSensorModel sensor_model;

  RobotAction action = 1.0;
  SensorReading z;

  RobotPose state;  // position and velocity
  auto state_belief = std::make_shared<GaussianMoment<RobotPose>>(state,
      Matrix(3, 3, {0, 0, 0, 0, 0, 0, 0, 0, 1000}));

  beliefs.push_back(state_belief);
  for (int i = 0; i < steps; i++) {
    state_belief = DoTimeUpdate(*state_belief, action, action_model);
    beliefs.push_back(state_belief);
    state_belief = DoMeasurementUpdate(*state_belief, z, q, sensor_model);
    beliefs.push_back(state_belief);
  }

  for (unsigned int i = 0; i < beliefs.size(); i++) {
    std::cout << *beliefs[i] << std::endl;
  }
}


}  // namespace examples
