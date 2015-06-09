/*
 * Position1dKalmanFilter.cc
 *
 *  Based on exercise 3.1 from "Probabilistic Robotics" by Fox, Thrun,
 *  and Burgard.
 *
 *  Copyright (c) 2015, Norman Alan Oursland
 *  All rights reserved.
 */

#include "cognitoware/math/data/Matrix.h"
#include "cognitoware/math/data/Vector.h"
#include "cognitoware/math/probability/continuous/GaussianMoment.h"
#include "cognitoware/robotics/state_estimation/KalmanFilter.h"
#include "gtest/gtest.h"

#include <iostream>
#include <memory>
#include <vector>

using ::cognitoware::math::data::Matrix;
using ::cognitoware::math::data::Vector;
using ::cognitoware::math::probability::continuous::GaussianMoment;
using ::cognitoware::robotics::state_estimation::KalmanActionModel;
using ::cognitoware::robotics::state_estimation::KalmanSensorModel;
using ::cognitoware::robotics::state_estimation::KalmanFilter;

namespace examples {
namespace {
DEFINE_VECTOR1(Position);
DEFINE_VECTOR1(Velocity);
DEFINE_VECTOR1(Acceleration);

class State : public Vector {
public:
  State() :
      Vector({0.0, 0.0}) {
  }
  State(double pos, double vel) :
      Vector({pos, vel}) {
  }
  State(const State& that) :
      Vector(that) {
  }
  State(State&& that) :
      Vector(that) {
  }
};

class SensorReading : public Vector {
public:
  SensorReading() :
      Vector({0, 0}) {
  }
  SensorReading(double pos, double vel) :
      Vector({pos, vel}) {
  }
  SensorReading(const SensorReading& that) :
      Vector(that) {
  }
  SensorReading(SensorReading&& that) :
      Vector(that) {
  }
};

typedef KalmanActionModel<Acceleration, State> ActionModel;
typedef KalmanSensorModel<SensorReading, State> SensorModel;

std::vector<double> ma(std::vector<double> result) {
  return result;
}

std::ostream& operator<<(std::ostream& os, const GaussianMoment<State>& x) {
  const State& mu = x.mean();
  const Matrix& sigma = x.covariance();
  os << "mean(pos=" << mu.at(0) << ", vel=" << mu.at(1) << "), cov("
      << sigma.at(0, 0) << ", " << sigma.at(0, 1) << ", " << sigma.at(1, 0)
      << ", " << sigma.at(1, 1) << ")";
  return os;
}

std::shared_ptr<ActionModel> CreateActionModel() {
  auto action_model = std::make_shared<ActionModel>(2, 1);
  action_model->a() = Matrix(2, 2, ma({1.0, 1.0, 0.0, 1.0}));
  // position = position+velocity.
  // velocity = velocity
  action_model->b() = Matrix(2, 1, ma({0.0, 1.0}));
  // position = 0
  // velocity = acceleration
  action_model->c() = Vector(ma({0.0, 0.0}));
  action_model->r() = Matrix(2, 2, ma({0.0, 0.0, 0.0, 0.1}));
  return action_model;
}

std::shared_ptr<SensorModel> CreateSensorModel() {
  auto sensor_model = std::make_shared<SensorModel>(2, 1);
  sensor_model->c() = Matrix(2, 2, ma({1, 0, 0, 1}));
  sensor_model->d() = Vector({0, 0});
  // our velocity sensors are very bad
  sensor_model->q() = Matrix(2, 2, ma({10, 0, 0, 10000}));
  return sensor_model;
}
}  // namespace

TEST(Position1dKalmanFilter, ActionUpdate) {
  int steps = 6;
  std::vector<std::shared_ptr<GaussianMoment<State>>>beliefs(steps);

  State state;  // position and velocity
  GaussianMoment<Acceleration> action(Acceleration(0.0),
      Matrix(1, 1, ma({1.0})));
  auto belief = std::make_shared<GaussianMoment<State>>(state, Matrix(2, 2, ma({
      0.0, 0.0, 0.0, 0.0})));
  auto action_model = CreateActionModel();

  auto filter = std::make_shared<KalmanFilter<State, Acceleration, Vector>>();

  std::default_random_engine generator(0);

  beliefs[0] = belief;
  for (unsigned int i = 1; i < beliefs.size(); i++) {
    belief = filter->Marginalize(*action_model, action.Sample(&generator),
        *belief);
    beliefs[i] = belief;
  }

  for (unsigned int i = 0; i < beliefs.size(); i++) {
    std::cout << *beliefs[i] << std::endl;
  }
}

TEST(Position1dKalmanFilter, ObservationUpdate) {
  int steps = 6;
  std::vector<std::shared_ptr<GaussianMoment<State>>>beliefs;

  State state;  // position and velocity
  GaussianMoment<Acceleration> action(Acceleration(0.0),
      Matrix(1, 1, ma({1.0})));
  auto belief = std::make_shared<GaussianMoment<State>>(state, Matrix(2, 2, ma({
      0.0, 0.0, 0.0, 0.0})));
  auto action_model = CreateActionModel();
  auto sensor_model = CreateSensorModel();

  auto filter = std::make_shared<
      KalmanFilter<State, Acceleration, SensorReading>>();

  std::default_random_engine generator(0);

  beliefs.push_back(belief);
  for (int i = 1; i < steps; i++) {
    belief = filter->Marginalize(*action_model, action.Sample(&generator),
        *belief);
    beliefs.push_back(belief);
  }

  SensorReading z(5.0, 0.0);
  belief = filter->BayesianInference(*sensor_model, z, *belief);
  beliefs.push_back(belief);

  for (unsigned int i = 0; i < beliefs.size(); i++) {
    std::cout << *beliefs[i] << std::endl;
  }
}

}  // namespace examples
