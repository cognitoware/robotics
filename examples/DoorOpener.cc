/*
 * DoorOpener.cc
 *
 *  Based on example 2.4.2 from "Probabilistic Robotics" by Fox, Thrun,
 *  and Burgard.
 *
 *  Copyright (c) 2015, Norman Alan Oursland
 *  All rights reserved.
 */

#include "cognitoware/math/data/Vector.h"
#include "cognitoware/math/probability/discrete/ConditionalValueMap.h"
#include "cognitoware/math/probability/discrete/DistributionValueMap.h"
#include "cognitoware/math/probability/RandomConditional.h"
#include "cognitoware/math/probability/RandomDistribution.h"
#include "cognitoware/math/probability/IndependentPairDistribution.h"
#include "cognitoware/robotics/MarkovActionChain.h"
#include "gtest/gtest.h"

#include <memory>
#include <vector>

using ::cognitoware::robotics::MarkovActionChain;
using ::cognitoware::math::probability::IndependentPairDistribution;
using ::cognitoware::math::probability::RandomConditional;
using ::cognitoware::math::probability::RandomDistribution;
using ::cognitoware::math::probability::discrete::ConditionalValueMap;
using ::cognitoware::math::probability::discrete::DistributionValueMap;
using ::cognitoware::math::data::Vector;

namespace examples {

enum class DoorAction {
  Idle, Push
};

std::ostream& operator<<(std::ostream& os, DoorAction z) {
  switch (z) {
  case DoorAction::Idle:
    os << "Idle";
  break;
  case DoorAction::Push:
    os << "Push";
  break;
  }
  return os;
}

enum class DoorObservation {
  Open, Closed
};

std::ostream& operator<<(std::ostream& os, DoorObservation z) {
  switch (z) {
  case DoorObservation::Open:
    os << "Open";
  break;
  case DoorObservation::Closed:
    os << "Closed";
  break;
  }
  return os;
}

enum class DoorState {
  Open, Closed
};

typedef std::pair<DoorState, DoorAction> XU;

std::ostream& operator<<(std::ostream& os, DoorState x) {
  switch (x) {
  case DoorState::Open:
    os << "Open";
  break;
  case DoorState::Closed:
    os << "Closed";
  break;
  }
  return os;
}

std::ostream& operator<<(std::ostream& os,
                         const RandomDistribution<DoorState>& belief) {
  os << DoorState::Open << "=" << belief.ProbabilityOf(DoorState::Open);
  os << ", ";
  os << DoorState::Closed << "=" << belief.ProbabilityOf(DoorState::Closed);
  return os;
}

class Door {
public:
  Door(std::default_random_engine* generator) {
    p_ = 0.0;
    state_ = DoorState::Open;
    pxz_ = std::make_shared<ConditionalValueMap<DoorState, DoorObservation>>();
    pxux_ = std::make_shared<MarkovActionChain<DoorState, DoorAction>>();
    InitState(generator);
    pxz_->Set(DoorState::Closed, DoorObservation::Open, 0.2);
    pxz_->Set(DoorState::Open, DoorObservation::Open, 0.8);
    pxz_->Set(DoorState::Closed, DoorObservation::Closed, 0.6);
    pxz_->Set(DoorState::Open, DoorObservation::Closed, 0.4);

    pxux_->Set(DoorState::Closed, DoorAction::Push, DoorState::Closed, 0.2);
    pxux_->Set(DoorState::Open, DoorAction::Push, DoorState::Closed, 0.8);
    pxux_->Set(DoorState::Closed, DoorAction::Push, DoorState::Open, 0.0);
    pxux_->Set(DoorState::Open, DoorAction::Push, DoorState::Open, 1.0);
    pxux_->Set(DoorState::Closed, DoorAction::Idle, DoorState::Closed, 1.0);
    pxux_->Set(DoorState::Open, DoorAction::Idle, DoorState::Closed, 0.0);
    pxux_->Set(DoorState::Closed, DoorAction::Idle, DoorState::Open, 0.0);
    pxux_->Set(DoorState::Open, DoorAction::Idle, DoorState::Open, 1.0);
  }

  void InitState(std::default_random_engine* generator) {
    std::uniform_real_distribution<double> random(0, 1);
    double select = random(*generator);
    if (select < p_) {
      state_ = DoorState::Open;
    } else {
      state_ = DoorState::Closed;
    }
  }

  void CreateObservation(std::default_random_engine* generator,
                         DoorObservation* result_out) const {
    std::uniform_real_distribution<double> random(0, 1);
    *result_out = pxz_->SampleLikelihood(state_, random(*generator));
  }

  void DoAction(std::default_random_engine* generator, DoorAction u) {
    std::uniform_real_distribution<double> random(0, 1);
    state_ = pxux_->SampleCondition(XU(state_, u), random(*generator));
  }

private:
  double p_;
  DoorState state_;
  std::shared_ptr<ConditionalValueMap<DoorState, DoorObservation>> pxz_;
  std::shared_ptr<MarkovActionChain<DoorState, DoorAction>> pxux_;
};

class DoorActionModel : public MarkovActionChain<DoorState, DoorAction> {
public:
  DoorActionModel() {
    DoorState O = DoorState::Open;
    DoorState C = DoorState::Closed;
    DoorAction I = DoorAction::Idle;
    DoorAction P = DoorAction::Push;
    Set(O, P, O, 1.0);
    Set(C, P, O, 0.0);
    Set(O, P, C, 0.8);
    Set(C, P, C, 0.2);
    Set(O, I, O, 1.0);
    Set(C, I, O, 0.0);
    Set(O, I, C, 0.0);
    Set(C, I, C, 1.0);
  }
};

class DoorSensorModel : public ConditionalValueMap<DoorObservation, DoorState> {
public:
  DoorSensorModel() {
    Set(DoorObservation::Open, DoorState::Open, 0.6);
    Set(DoorObservation::Closed, DoorState::Open, 0.4);
    Set(DoorObservation::Open, DoorState::Closed, 0.2);
    Set(DoorObservation::Closed, DoorState::Closed, 0.8);
  }
};

std::shared_ptr<RandomDistribution<std::pair<DoorState, DoorAction>>> MakePairDistribution(
    std::shared_ptr<RandomDistribution<DoorState>> x,
    const DoorAction& u) {
  return std::make_shared<IndependentPairDistribution<DoorState, DoorAction>>(
      x,
      std::make_shared<DistributionValueMap<DoorAction>>(
          std::vector<DoorAction> {u}));
}

DoorAction ChooseAction(const RandomDistribution<DoorState>& x) {
  // decision is the 'core' of the AI
  if (x.ProbabilityOf(DoorState::Closed) > 0.9) {
    return DoorAction::Push;
  }
  return DoorAction::Idle;
}

TEST(DoorOpener, main) {
  std::default_random_engine generator(0);
  Door door(&generator);
  auto sensor_model = std::make_shared<DoorSensorModel>();
  auto action_model = std::make_shared<DoorActionModel>();
  auto x = std::make_shared<DistributionValueMap<DoorState>>(DoorState::Open,
      0.5, DoorState::Closed, 0.5);
  std::cout << "Start belief " << *x << std::endl << std::endl;

  for (int i = 0; i < 10; i++) {
    DoorObservation z;
    door.CreateObservation(&generator, &z);
    DoorAction u = ChooseAction(*x);
    x = sensor_model->BayesianInference(z, *x);
    std::cout << "Observed " << z << std::endl;
    std::cout << "After Observation " << *x << std::endl;
    auto xu = MakePairDistribution(x, u);
    x = action_model->Marginalize(*xu);
    std::cout << "Do " << u << std::endl;
    std::cout << "After Action " << *x << std::endl;
    door.DoAction(&generator, u);
    std::cout << std::endl;
  }
  std::cout << "Final belief " << *x << std::endl;
}

}  // namespace examples

