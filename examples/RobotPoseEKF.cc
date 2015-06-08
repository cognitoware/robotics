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
#include "cognitoware/robotics/state_estimation/ExtendedKalmanFilter.h"
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
using ::cognitoware::robotics::state_estimation::ExtendedKalmanFilter;
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
    //     d(y+u*Sin(a))/dx      d(y+u*Sin(a))/dy     d(y+u*Sin(a))/da
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

  Matrix GetJacobian(const RobotPose&) const override {
    // g = d(x)/dx  d(x)/dy  d(x)/da
    // = 1  0  0
    return Matrix( 1, 3, {1, 0, 0});
  }

  Matrix q_;
};

std::shared_ptr<GaussianMoment<RobotPose>> DoTimeUpdate(
    const GaussianMoment<RobotPose>& state_belief,
    const RobotAction& u,
    const RobotActionModel& action_model) {
  RobotPose x0 = state_belief.mean();
  std::cout << "\t" << "x0 = " << x0 << std::endl;
  Matrix e0 = state_belief.covariance();
  std::cout << "\t" << "e0 = " << e0 << std::endl;

  // time update
  Matrix g = action_model.GetStateJacobian(u, x0);
  std::cout << "\t" << "g = " << g << std::endl;
  Matrix gt = g.Transpose();
  std::cout << "\t" << "gt = " << gt << std::endl;

  RobotPose x1 = action_model.GetMean(u, x0);
  std::cout << "\t" << "x1 = " << x1 << std::endl;
  Matrix r = action_model.GetError(u, x0);
  std::cout << "\t" << "r = " << r << std::endl;
  Matrix e1 = g * e0 * gt + r;
  std::cout << "\t" << "e1 = " << e1 << std::endl;

  return std::make_shared<GaussianMoment<RobotPose>>(x1, e1);
}

std::shared_ptr<GaussianMoment<RobotPose>> DoMeasurementUpdate(
    const GaussianMoment<RobotPose>& state_belief,
    const SensorReading& z,
    const RobotSensorModel& sensor_model) {
  std::cout << "\t" << "state_belief = " << state_belief << std::endl;
  std::cout << "\t" << "z = " << z << std::endl;

  // measurement update
  RobotPose x0 = state_belief.mean();
  std::cout << "\t" << "x0 = " << x0 << std::endl;
  Matrix e0 = state_belief.covariance();
  std::cout << "\t" << "e0 = " << e0 << std::endl;

  Matrix h = sensor_model.GetJacobian(x0);
  std::cout << "\t" << "h = " << h << std::endl;
  Matrix ht = h.Transpose();
  std::cout << "\t" << "ht = " << ht << std::endl;

  Matrix i = Matrix::Identity(e0.order());
  std::cout << "\t" << "i = " << i << std::endl;

  SensorReading expected_z = sensor_model.GetMean(x0);
  const Matrix& q = sensor_model.GetError(z);

  Matrix HEHt_Q = h * e0 * ht + q;
  std::cout << "\t" << "HE = " << h * e0 << std::endl;
  std::cout << "\t" << "HEHt = " << h * e0 * ht << std::endl;
  std::cout << "\t" << "HEHt_Q = " << HEHt_Q << std::endl;

  Matrix k = e0 * ht * (h * e0 * ht + q).Inverse();
  std::cout << "\t" << "k = " << k << std::endl;
  Vector v = (x0 + k * (z - expected_z));
  std::cout << "\t" << "v = " << v << std::endl;
  RobotPose x1(v[0], v[1], v[2]);
  std::cout << "\t" << "x1 = " << x1 << std::endl;
  Matrix e1 = (i - k * h) * e0;
  std::cout << "\t" << "e1 = " << e1 << std::endl;

  return std::make_shared<GaussianMoment<RobotPose>>(x1, e1);
}
}

TEST(RobotPoseEKF, ActionObsevatonUpdate) {
  int steps = 6;

  RobotActionModel action_model;
  RobotSensorModel sensor_model;

  RobotAction action = 1.0;
  SensorReading z(0.0);

  // We know x and y perfectly, but we don't know the heading at all.
  // This results in highly non-linear behavior that the EKF does not
  // approximate well.
  auto state_belief = std::make_shared<GaussianMoment<RobotPose>>(RobotPose(),
      Matrix(3, 3, {0, 0, 0, 0, 0, 0, 0, 0, 1000}));

  std::cout << "Start: " << *state_belief << std::endl;
  for (int i = 0; i < steps; i++) {
    state_belief = DoTimeUpdate(*state_belief, action, action_model);
    std::cout << "Act: " << *state_belief << std::endl;
    // The EKF is 100% certain of the belief x-coordinate so it ignores
    // the sensor reading.
    state_belief = DoMeasurementUpdate(*state_belief, z, sensor_model);
    std::cout << "Observe: " << *state_belief << std::endl;
  }
  std::cout << "End: " << *state_belief << std::endl;
}

TEST(RobotPoseEKF, EKF) {
  int steps = 6;

  RobotActionModel action_model;
  RobotSensorModel sensor_model;

  RobotAction action = 1.0;
  SensorReading z(0.0);

  // We know x and y perfectly, but we don't know the heading at all.
  // This results in highly non-linear behavior that the EKF does not
  // approximate well.
  auto state_belief = std::make_shared<GaussianMoment<RobotPose>>(RobotPose(),
      Matrix(3, 3, {0, 0, 0, 0, 0, 0, 0, 0, 1000}));


  ExtendedKalmanFilter<RobotPose, RobotAction, SensorReading> ekf;
  std::cout << "Start: " << *state_belief << std::endl;
  for (int i = 0; i < steps; i++) {
    state_belief = ekf.Marginalize(action_model, action, *state_belief);
    std::cout << "Act: " << *state_belief << std::endl;
    // The EKF is 100% certain of the belief x-coordinate so it ignores
    // the sensor reading.
    state_belief = ekf.BayesianInference(sensor_model, z, *state_belief);
    std::cout << "Observe: " << *state_belief << std::endl;
  }
  std::cout << "End: " << *state_belief << std::endl;
}


}  // namespace examples
