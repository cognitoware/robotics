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
#include "cognitoware/robotics/state_estimation/ExtendedInformationFilter.h"
#include "cognitoware/robotics/state_estimation/ExtendedKalmanFilter.h"
#include "cognitoware/robotics/state_estimation/InformationFilter.h"
#include "cognitoware/robotics/state_estimation/KalmanFilter.h"
#include "cognitoware/robotics/state_estimation/UnscentedKalmanFilter.h"
#include "gtest/gtest.h"

#include <cmath>
#include <iostream>
#include <memory>
#include <vector>

using ::cognitoware::math::data::Matrix;
using ::cognitoware::math::data::Vector;
using ::cognitoware::math::probability::continuous::GaussianCanonical;
using ::cognitoware::math::probability::continuous::GaussianMoment;
using ::cognitoware::math::probability::RandomDistribution;
using ::cognitoware::robotics::state_estimation::ExtendedInformationFilter;
using ::cognitoware::robotics::state_estimation::ExtendedKalmanActionModel;
using ::cognitoware::robotics::state_estimation::ExtendedKalmanFilter;
using ::cognitoware::robotics::state_estimation::ExtendedKalmanSensorModel;
using ::cognitoware::robotics::state_estimation::GaussianActionModel;
using ::cognitoware::robotics::state_estimation::GaussianSensorModel;
using ::cognitoware::robotics::state_estimation::InformationFilter;
using ::cognitoware::robotics::state_estimation::KalmanActionModel;
using ::cognitoware::robotics::state_estimation::KalmanFilter;
using ::cognitoware::robotics::state_estimation::KalmanSensorModel;
using ::cognitoware::robotics::state_estimation::UnscentedKalmanFilter;

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

  void UpdateState(const U& u) {
    double u_err = action_noise_.Sample(&generator_)[0];
    double actual_u = u[0] + u[0]*u_err;
    double process_err = process_noise_.Sample(&generator_)[0];
    double dx = actual_state_[0] + actual_u;
    actual_state_ = X(dx + dx*process_err);
  }

  X UpdateState(const X& x, const U& u) {
    UpdateState(u);

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

class EKFActionModel : public ExtendedKalmanActionModel<U, X> {
public:
  EKFActionModel() : r_(1, 1) {}

  Matrix GetStateJacobian(const U&,
                          const X&) const override {
    // x_next = x + u
    return Matrix(1, 1, {1});
  }
  Matrix GetActionJacobian(const U&,
                           const X&) const override {
    return Matrix(1, 1, {1});
  }

  X GetMean(const U& u, const X& x) const override {
    return X(x[0] + u[0]);
  }

  Matrix GetError(const U&, const X&) const override {
    return r_;
  }

  std::shared_ptr<RandomDistribution<X>> ConditionBy(
      const U& u, const X& x) {
    return std::make_shared<GaussianMoment<X>>(
        GetMean(u, x),
        GetError(u, x));
  }

  Matrix r_;
};

class EKFSensorModel : public ExtendedKalmanSensorModel<Z, X> {
public:
  EKFSensorModel() : q_(1, 1, {0.01}) {
  }

  double ConditionalProbabilityOf(const Z& observation,
                                  const X& state) const override {
    GaussianMoment<X> likelihood(state, q_);
    X observed_x(10.0 - observation[0]);
    return likelihood.ProbabilityOf(observed_x);
  }

  Z GetMean(const X& state) const override {
    return Z(10.0 - state[0]);
  }

  Matrix GetError(const Z&) const override {
    return q_;
  }

  Matrix GetJacobian(const X&) const override {
    // z = 10 - x
    // x/dx = -1
    return Matrix(1, 1, {-1});
  }

  Matrix q_;
};

typedef EKFActionModel UKFActionModel;
typedef EKFSensorModel UKFSensorModel;
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
  std::cout << "End: " << state << " (actual " << sim.actual_state_ << ")" << std::endl;
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
  std::cout << "End: " << state_belief << " (actual " << sim.actual_state_ << ")" << std::endl;
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
  std::cout << "End: " << state_belief << " (actual " << sim.actual_state_ << ")" << std::endl;
}

TEST(LocalizationParametricFilterRoundup, KalmanFilter) {
  RobotSimulator sim;
  sim.action_noise_ = GaussianMoment<U>(U(0.0), Matrix(1, 1, {0.0004}));
  sim.sensor_noise_ = GaussianMoment<Z>(Z(0.0), Matrix(1, 1, {0.0016}));
  sim.process_noise_ = GaussianMoment<X>(Z(0.0), Matrix(1, 1, {0.0004}));

  KalmanActionModel<U, X> action_model(1, 1);
  action_model.a() = Matrix(1,1,{1.0});
  action_model.b() = Matrix(1,1,{1.0});
  action_model.c() = Vector(std::vector<double>({0.0}));
  // We overestimate the error because over-fitting is worse than under-fitting.
  action_model.r() = 2*(sim.action_noise_.covariance() + sim.process_noise_.covariance());

  KalmanSensorModel<Z, X> sensor_model(1, 1);
  // setup the sensor model to convert the observation to an expected state
  sensor_model.c() = Matrix(1,1,{-1.0});
  sensor_model.d().Set(sim.landmark_position_.AliasVector());
  // Again, we over-estimate the error.
  sensor_model.q() = 2*sim.sensor_noise_.covariance();

  KalmanFilter<X, U, Z> filter;

  // We know with perfect certainty that we start at 0.0
  auto state_belief = std::make_shared<GaussianMoment<X>>(0.0, Matrix(1, 1, {0}));
  std::cout << "Start: " << *state_belief << std::endl;
  for(const U& action : actions) {
    sim.UpdateState(action);
    state_belief = filter.Marginalize(action_model, action, *state_belief);
    std::cout << "Act: " << *state_belief << " (actual " << sim.actual_state_ << ")" << std::endl;
    Z observation = sim.ObserveLandmark();
    state_belief = filter.BayesianInference(sensor_model, observation, *state_belief);
    std::cout << "See: " << *state_belief << " (actual " << sim.actual_state_ << ")" << std::endl;
  }
  std::cout << "End: " << *state_belief << " (actual " << sim.actual_state_ << ")" << std::endl;
}

TEST(LocalizationParametricFilterRoundup, ExtendedKalmanFilter) {
  RobotSimulator sim;
  sim.action_noise_ = GaussianMoment<U>(U(0.0), Matrix(1, 1, {0.0004}));
  sim.sensor_noise_ = GaussianMoment<Z>(Z(0.0), Matrix(1, 1, {0.0016}));
  sim.process_noise_ = GaussianMoment<X>(Z(0.0), Matrix(1, 1, {0.0004}));

  EKFActionModel action_model;
  action_model.r_ = 2*(sim.action_noise_.covariance() + sim.process_noise_.covariance());
  EKFSensorModel sensor_model;
  sensor_model.q_ = 2*sim.sensor_noise_.covariance();

  ExtendedKalmanFilter<X, U, Z> filter;

  // We know with perfect certainty that we start at 0.0
  auto state_belief = std::make_shared<GaussianMoment<X>>(0.0, Matrix(1, 1, {0}));
  std::cout << "Start: " << *state_belief << std::endl;
  for(const U& action : actions) {
    sim.UpdateState(action);
    state_belief = filter.Marginalize(action_model, action, *state_belief);
    std::cout << "Act: " << *state_belief << " (actual " << sim.actual_state_ << ")" << std::endl;
    Z observation = sim.ObserveLandmark();
    state_belief = filter.BayesianInference(sensor_model, observation, *state_belief);
    std::cout << "See: " << *state_belief << " (actual " << sim.actual_state_ << ")" << std::endl;
  }
  std::cout << "End: " << *state_belief << " (actual " << sim.actual_state_ << ")" << std::endl;
}

TEST(LocalizationParametricFilterRoundup, UnscentedKalmanFilter) {
  RobotSimulator sim;
  sim.action_noise_ = GaussianMoment<U>(U(0.0), Matrix(1, 1, {0.0004}));
  sim.sensor_noise_ = GaussianMoment<Z>(Z(0.0), Matrix(1, 1, {0.0016}));
  sim.process_noise_ = GaussianMoment<X>(Z(0.0), Matrix(1, 1, {0.0004}));

  UKFActionModel action_model;
  action_model.r_ = 2*(sim.action_noise_.covariance() + sim.process_noise_.covariance());
  UKFSensorModel sensor_model;
  sensor_model.q_ = 2*sim.sensor_noise_.covariance();

  UnscentedKalmanFilter<X, U, Z> filter(0.001, 2.0, 0.0);

  // We know with perfect certainty that we start at 0.0
  auto state_belief = std::make_shared<GaussianMoment<X>>(0.0, Matrix(1, 1, {0}));
  std::cout << "Start: " << *state_belief << std::endl;
  for(const U& action : actions) {
    sim.UpdateState(action);
    state_belief = filter.Marginalize(action_model, action, *state_belief);
    std::cout << "Act: " << *state_belief << " (actual " << sim.actual_state_ << ")" << std::endl;
    Z observation = sim.ObserveLandmark();
    state_belief = filter.BayesianInference(sensor_model, observation, *state_belief);
    std::cout << "See: " << *state_belief << " (actual " << sim.actual_state_ << ")" << std::endl;
  }
  std::cout << "End: " << *state_belief << " (actual " << sim.actual_state_ << ")" << std::endl;
}

TEST(LocalizationParametricFilterRoundup, InformationFilter) {
  RobotSimulator sim;
  sim.action_noise_ = GaussianMoment<U>(U(0.0), Matrix(1, 1, {0.0004}));
  sim.sensor_noise_ = GaussianMoment<Z>(Z(0.0), Matrix(1, 1, {0.0016}));
  sim.process_noise_ = GaussianMoment<X>(Z(0.0), Matrix(1, 1, {0.0004}));

  KalmanActionModel<U, X> action_model(1, 1);
  action_model.a() = Matrix(1,1,{1.0});
  action_model.b() = Matrix(1,1,{1.0});
  action_model.c() = Vector(std::vector<double>({0.0}));
  // We overestimate the error because over-fitting is worse than under-fitting.
  action_model.r() = 2*(sim.action_noise_.covariance() + sim.process_noise_.covariance());

  KalmanSensorModel<Z, X> sensor_model(1, 1);
  // setup the sensor model to convert the observation to an expected state
  sensor_model.c() = Matrix(1,1,{-1.0});
  sensor_model.d().Set(sim.landmark_position_.AliasVector());
  // Again, we over-estimate the error.
  sensor_model.q() = 2*sim.sensor_noise_.covariance();

  InformationFilter<X, U, Z> filter;

  // We know with perfect certainty that we start at 0.0
  auto state_belief = std::make_shared<GaussianCanonical<X>>(
      Vector(std::vector<double>({0.0})),
      Matrix(1, 1, {1}));
  std::cout << "Start: " << *state_belief << std::endl;
  for(const U& action : actions) {
    sim.UpdateState(action);
    state_belief = filter.Marginalize(action_model, action, *state_belief);
    std::cout << "Act: " << *state_belief << " (actual " << sim.actual_state_ << ")" << std::endl;
    Z observation = sim.ObserveLandmark();
    state_belief = filter.BayesianInference(sensor_model, observation, *state_belief);
    std::cout << "See: " << *state_belief << " (actual " << sim.actual_state_ << ")" << std::endl;
  }
  std::cout << "End: " << *state_belief << " (actual " << sim.actual_state_ << ")" << std::endl;
}

TEST(LocalizationParametricFilterRoundup, ExtendedInformationFilter) {
  RobotSimulator sim;
  sim.action_noise_ = GaussianMoment<U>(U(0.0), Matrix(1, 1, {0.0004}));
  sim.sensor_noise_ = GaussianMoment<Z>(Z(0.0), Matrix(1, 1, {0.0016}));
  sim.process_noise_ = GaussianMoment<X>(Z(0.0), Matrix(1, 1, {0.0004}));

  EKFActionModel action_model;
  action_model.r_ = 2*(sim.action_noise_.covariance() + sim.process_noise_.covariance());
  EKFSensorModel sensor_model;
  sensor_model.q_ = 2*sim.sensor_noise_.covariance();

  ExtendedInformationFilter<X, U, Z> filter;

  // We know with perfect certainty that we start at 0.0
  auto state_belief = std::make_shared<GaussianCanonical<X>>(
      Vector(std::vector<double>({0.0})),
      Matrix(1, 1, {1}));
  std::cout << "Start: " << *state_belief << std::endl;
  for(const U& action : actions) {
    sim.UpdateState(action);
    state_belief = filter.Marginalize(action_model, action, *state_belief);
    std::cout << "Act: " << *state_belief << " (actual " << sim.actual_state_ << ")" << std::endl;
    Z observation = sim.ObserveLandmark();
    state_belief = filter.BayesianInference(sensor_model, observation, *state_belief);
    std::cout << "See: " << *state_belief << " (actual " << sim.actual_state_ << ")" << std::endl;
  }
  std::cout << "End: " << *state_belief << " (actual " << sim.actual_state_ << ")" << std::endl;
}

}  // namespace examples


