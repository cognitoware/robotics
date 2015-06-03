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
#include "cognitoware/robotics/state_estimation/KalmanActionModel.h"
#include "cognitoware/robotics/state_estimation/KalmanFilter.h"
#include "gtest/gtest.h"

#include <iostream>
#include <memory>
#include <vector>

using ::cognitoware::math::data::Matrix;
using ::cognitoware::math::data::Vector;
using ::cognitoware::math::probability::continuous::GaussianMoment;
using ::cognitoware::robotics::state_estimation::KalmanActionModel;
using ::cognitoware::robotics::state_estimation::KalmanFilter;

namespace examples {

DEFINE_VECTOR1(Position);
DEFINE_VECTOR1(Velocity);
DEFINE_VECTOR1(Acceleration);

class State : public Vector {
public:
  State() :
      Vector({0.0, 0.0}) {
  }
  State(const State& that) :
      Vector(that) {
  }
  State(State&& that) :
      Vector(that) {
  }
  State(double pos, double vel) :
      Vector({pos, vel}) {
  }
};

typedef KalmanActionModel<Acceleration, State> ActionModel;

std::vector<double> ma(std::vector<double> result) {
  return result;
}

std::ostream& operator<<(std::ostream& os, const GaussianMoment<State>& x) {
  const State& mu = x.mean();
  const Matrix& sigma = x.covariance();
  os << "mean(pos=" << mu.at(0) << ", vel=" << mu.at(1) << "), cov(" << sigma.at(0, 0)
      << ", " << sigma.at(0, 1) << ", " << sigma.at(1, 0) << ", "
      << sigma.at(1, 1) << ")";
  return os;
}

std::shared_ptr<ActionModel> CreateActionModel() {
  auto action_model = std::make_shared<ActionModel>();
  action_model->a() = Matrix(2, 2, ma({1.0, 1.0, 0.0, 1.0}));
  // position = position+velocity.
  // velocity = velocity
  action_model->b() = Matrix(2, 1, ma({0.0, 1.0}));
  // position = 0
  // velocity = acceleration
  action_model->c() = Vector(ma({0.0, 0.0}));
  action_model->r() = Matrix(2, 2, ma({0.0, 0.0, 0.0, 0.0}));
  return action_model;
}

TEST(Position1dKalmanFilter, main) {
  int steps = 6;
  std::vector<std::shared_ptr<GaussianMoment<State>>> beliefs(steps);

  GaussianMoment<Acceleration> action(
      Acceleration(1.0),
      Matrix(1, 1, ma( {0.2})));

  State state;  // position and velocity
  auto belief = std::make_shared<GaussianMoment<State>>(
      state,
      Matrix(2, 2, ma( {0.0, 0.0, 0.0, 0.0})));
  auto action_model = CreateActionModel();

  auto filter = std::make_shared<KalmanFilter<State, Acceleration, Vector>>();

  beliefs[0] = belief;
  for( unsigned int i = 1; i < beliefs.size(); i++ ) {
    belief = filter->Marginalize(*action_model, action, *belief);
    beliefs[i] = belief;
  }

  for (unsigned int i = 0; i < beliefs.size(); i++) {
    std::cout << *beliefs[i] << std::endl;
  }
}

}  // namespace examples
