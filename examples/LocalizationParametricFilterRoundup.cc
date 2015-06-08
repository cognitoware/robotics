/*
 * LocalizationParametricFilterRoundup.cc
 *
 *  This example localizes a robot along a 1D axis against a landmark with a
 *  known position.
 *
 *  It does so with a Kalman Filter, Extended Kalman Filter, Unscented Kalman
 *  Filter, and an Information Filter.
 *
 *  Copyright (c) 2015, Norman Alan Oursland
 *  All rights reserved.
 */

#include "cognitoware/math/data/Matrix.h"
#include "cognitoware/math/data/Vector.h"
#include "cognitoware/math/probability/continuous/GaussianMoment.h"
#include "cognitoware/math/probability/RandomDistribution.h"
#include "cognitoware/robotics/state_estimation/ExtendedKalmanFilter.h"
#include "cognitoware/robotics/state_estimation/KalmanFilter.h"
#include "gtest/gtest.h"

#include <cmath>
#include <iostream>
#include <memory>
#include <vector>

using ::cognitoware::math::data::Matrix;
using ::cognitoware::math::data::Vector;
using ::cognitoware::math::probability::continuous::GaussianMoment;
using ::cognitoware::math::probability::RandomDistribution;
using ::cognitoware::robotics::state_estimation::ExtendedKalmanActionModel;
using ::cognitoware::robotics::state_estimation::ExtendedKalmanFilter;
using ::cognitoware::robotics::state_estimation::ExtendedKalmanSensorModel;
using ::cognitoware::robotics::state_estimation::KalmanActionModel;
using ::cognitoware::robotics::state_estimation::KalmanFilter;
using ::cognitoware::robotics::state_estimation::KalmanSensorModel;

namespace examples {
namespace {
// X holds a position.
DEFINE_VECTOR1(X);

// U holds the offset to move.
DEFINE_VECTOR1(U);
std::vector<U> actions({1.0, 1.5, -0.5, 2.0, 1.5, 1.0});

// Z holds a sensor reading
DEFINE_VECTOR1(Z);

class RobotSimulator {
public:

  X UpdateState(const X& x, const U& u) {
    double u_err = action_noise_.Sample(&generator_)[0];
    double actual_u = u[0] + u[0]*u_err;
    double process_err = process_noise_.Sample(&generator_)[0];
    double dx = actual_state_[0] + actual_u;
    actual_state_ = X(dx + dx*process_err);

    double expected_x = x[0];
    double expected_u = u[0];
    return X(expected_x + expected_u);
  }

  Z ObserveLandmark() {
    double z_err = sensor_noise_.Sample(&generator_)[0];
    double z = landmark_position_[0] - actual_state_[0];
    return Z(z + z*z_err);
  }

  X GetExpectedStateForObservation(const Z& z) {
    return X(landmark_position_[0] - z[0]);
  }

  std::default_random_engine generator_ = std::default_random_engine(0);

  X actual_state_ = X(0.0);
  X landmark_position_ = X(10.0);

  // The action the robot tries to perform ends up off by a bit.
  GaussianMoment<U> action_noise_ = GaussianMoment<U>(U(0.0), Matrix(1, 1, {0}));
  // The sensor reading the robot makes is off by a bit.
  GaussianMoment<Z> sensor_noise_ = GaussianMoment<Z>(Z(0.0), Matrix(1, 1, {0}));
  // The position resulting from an action is off by a bit.
  GaussianMoment<X> process_noise_ = GaussianMoment<X>(X(0.0), Matrix(1, 1, {0}));
};
}  // namespace

TEST(LocalizationParametricFilterRoundup, PerfectInformation) {
  RobotSimulator sim;

  X state(0.0);
  std::cout << "Start: " << state << std::endl;
  for(const U& action : actions) {
    state = sim.UpdateState(state, action);
    std::cout << "Act: " << state << std::endl;
    Z observation = sim.ObserveLandmark();
    X inferred_state = sim.GetExpectedStateForObservation(observation);
    // No filter needed. If state ever differs from inferred state then the world is broken.
    std::cout << "See: " << state << " (after observing " << inferred_state << ")" << std::endl;
  }
  std::cout << "End: " << state << std::endl;
}

TEST(LocalizationParametricFilterRoundup, NoiseNoObservation) {
  RobotSimulator sim;
  sim.action_noise_ = GaussianMoment<U>(U(0.0), Matrix(1, 1, {0.0004}));
  sim.sensor_noise_ = GaussianMoment<Z>(Z(0.0), Matrix(1, 1, {0.0016}));
  sim.process_noise_ = GaussianMoment<X>(Z(0.0), Matrix(1, 1, {0.0004}));

  X state_belief(0.0);
  std::cout << "Start: " << state_belief << std::endl;
  for(const U& action : actions) {
    state_belief = sim.UpdateState(state_belief, action);
    std::cout << "Act: " << state_belief << " (actual " << sim.actual_state_ << ")" << std::endl;
    sim.ObserveLandmark();
  }
  std::cout << "End: " << state_belief << std::endl;
}

TEST(LocalizationParametricFilterRoundup, FirstFilter) {
  RobotSimulator sim;
  sim.action_noise_ = GaussianMoment<U>(U(0.0), Matrix(1, 1, {0.0004}));
  sim.sensor_noise_ = GaussianMoment<Z>(Z(0.0), Matrix(1, 1, {0.0016}));
  sim.process_noise_ = GaussianMoment<X>(Z(0.0), Matrix(1, 1, {0.0004}));

  X state_belief(0.0);
  std::cout << "Start: " << state_belief << std::endl;
  for(const U& action : actions) {
    state_belief = sim.UpdateState(state_belief, action);
    std::cout << "Act: " << state_belief << " (actual " << sim.actual_state_ << ")" << std::endl;
    Z observation = sim.ObserveLandmark();
    X inferred_state = sim.GetExpectedStateForObservation(observation);
    // Cheap and easy filter: Weighted average of current belief and new observation.
    state_belief = X(0.67*state_belief[0] + 0.33*inferred_state[0]);
    std::cout << "See: " << state_belief << " (inferred " << inferred_state << ")" << std::endl;

  }
  std::cout << "End: " << state_belief << std::endl;
}



}  // namespace examples


