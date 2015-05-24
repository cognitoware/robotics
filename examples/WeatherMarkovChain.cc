/*
 * WeatherMarkovChain.cc
 *
 *  Based on exercise 2.2 from "Probabilistic Robotics" by Fox, Thrun,
 *  and Burgard.
 *
 *  Copyright (c) 2015, Norman Alan Oursland
 *  All rights reserved.
 */

#include "cognitoware/math/data/Vector.h"
#include "cognitoware/math/probability/ConditionalMap.h"
#include "cognitoware/math/probability/discrete/DistributionCounter.h"
#include "cognitoware/math/probability/discrete/DistributionValueMap.h"
#include "cognitoware/math/probability/discrete/MarkovChain.h"
#include "cognitoware/math/probability/RandomDistribution.h"
#include "cognitoware/math/probability/RangedUniform.h"
#include "gtest/gtest.h"

#include <iostream>
#include <random>
#include <vector>
#include <utility>

using ::cognitoware::math::data::Vector;
using ::cognitoware::math::probability::ConditionalMap;
using ::cognitoware::math::probability::discrete::DistributionCounter;
using ::cognitoware::math::probability::discrete::DistributionValueMap;
using ::cognitoware::math::probability::discrete::MarkovChain;
using ::cognitoware::math::probability::RandomDistribution;
using ::cognitoware::math::probability::RangedUniform;

namespace examples {

enum class Weather {
  Sunny, Cloudy, Rainy
};
std::ostream& operator<<(std::ostream& os, Weather w) {
  switch (w) {
  case Weather::Sunny:
    os << "Sunny";
  break;
  case Weather::Cloudy:
    os << "Cloudy";
  break;
  case Weather::Rainy:
    os << "Rainy";
  break;
  }
  return os;
}

std::ostream& operator<<(std::ostream& os,
                         const RandomDistribution<Weather>& belief) {
  os << Weather::Sunny << "=" << belief.ProbabilityOf(Weather::Sunny);
  os << ", ";
  os << Weather::Cloudy << "=" << belief.ProbabilityOf(Weather::Cloudy);
  os << ", ";
  os << Weather::Rainy << "=" << belief.ProbabilityOf(Weather::Rainy);
  return os;
}

std::shared_ptr<MarkovChain<Weather>> CreateWeatherMachine() {
  auto m = std::make_shared<MarkovChain<Weather>>();
  m->Set(Weather::Sunny, Weather::Sunny, 0.8);
  m->Set(Weather::Cloudy, Weather::Sunny, 0.2);
  m->Set(Weather::Rainy, Weather::Sunny, 0.0);
  m->Set(Weather::Sunny, Weather::Cloudy, 0.4);
  m->Set(Weather::Cloudy, Weather::Cloudy, 0.4);
  m->Set(Weather::Rainy, Weather::Cloudy, 0.2);
  m->Set(Weather::Sunny, Weather::Rainy, 0.2);
  m->Set(Weather::Cloudy, Weather::Rainy, 0.6);
  m->Set(Weather::Rainy, Weather::Rainy, 0.2);
  return m;
}

TEST(WeatherMarkovChain, TransitionProbabilites) {
  auto m = CreateWeatherMachine();
  std::vector<Weather> a = { Weather::Sunny, Weather::Cloudy, Weather::Cloudy,
      Weather::Rainy };
  double p = 1.0;
  std::cout << a[0] << "=" << p << std::endl;
  for (unsigned int i = 1; i < a.size(); i++) {
    p *= m->ConditionalProbabilityOf(a[i], a[i - 1]);
    std::cout << a[i] << "=" << p << std::endl;
  }
}

TEST(WeatherMarkovChain, TransitionSampling) {
  std::default_random_engine generator(0);
  auto m = CreateWeatherMachine();
  Weather w = Weather::Sunny;
  for (int i = 0; i < 50; i++) {
    w = m->DoTransition(w, &generator);
    // std::cout << w << std::endl;
  }
}

TEST(WeatherMarkovChain, MarginalizeOverNaiveBelief) {
  auto m = CreateWeatherMachine();
  std::vector<Weather> a = { Weather::Sunny, Weather::Cloudy, Weather::Cloudy,
      Weather::Rainy };
  auto belief = std::make_shared<DistributionValueMap<Weather>>(a);
  for (int i = 0; i <= 200; i++) {
    belief = m->Marginalize(*belief);
  }
  std::cout << "Belief: " << *belief << std::endl;
  std::cout << "Entropy: " << belief->GetEntropy() << std::endl;
}

TEST(WeatherMarkovChain, SunnyOutlook) {
  std::default_random_engine generator(0);
  Weather x = Weather::Sunny;
  auto m = CreateWeatherMachine();
  auto belief = std::make_shared<DistributionCounter<Weather>>();
  for (int i = 0; i < 500; i++) {
    x = m->DoTransition(x, &generator);
    belief->AddObservation(x);
  }
  std::cout << *belief << std::endl;
}

TEST(WeatherMarkovChain, Reverse) {
  auto m = CreateWeatherMachine();
  auto reverse = m->Reverse();
  for ( Weather today : m->domain() ) {
      std::cout << "If today is " << today << " yesterday was: ";
      std::cout << Weather::Sunny << "=" << reverse->ConditionalProbabilityOf(Weather::Sunny, today) << ", ";
      std::cout << Weather::Cloudy << "=" << reverse->ConditionalProbabilityOf(Weather::Cloudy, today) << ", ";
      std::cout << Weather::Rainy << "=" << reverse->ConditionalProbabilityOf(Weather::Rainy, today) << std::endl;
  }
}
}  // namespace examples

