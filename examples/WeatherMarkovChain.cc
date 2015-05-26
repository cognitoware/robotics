/*
 * WeatherMarkovChain.cc
 *
 *  Based on exercise 2.2 from "Probabilistic Robotics" by Fox, Thrun,
 *  and Burgard.
 *
 *  Copyright (c) 2015, Norman Alan Oursland
 *  All rights reserved.
 */

#include "cognitoware/math/probability/discrete/DistributionCounter.h"
#include "cognitoware/math/probability/discrete/DistributionValueMap.h"
#include "cognitoware/math/probability/discrete/MarkovChain.h"
#include "cognitoware/math/probability/RandomDistribution.h"
#include "examples/Weather.h"
#include "gtest/gtest.h"

#include <iostream>
#include <random>
#include <vector>
#include <utility>

using ::cognitoware::math::probability::discrete::DistributionCounter;
using ::cognitoware::math::probability::discrete::DistributionValueMap;
using ::cognitoware::math::probability::discrete::MarkovChain;
using ::cognitoware::math::probability::RandomDistribution;

namespace examples {

TEST(WeatherMarkovChain, TransitionProbabilites) {
  auto m = CreateWeatherMachine();
  std::vector<Weather> a = {Weather::Sunny, Weather::Cloudy, Weather::Cloudy,
      Weather::Rainy};
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
  std::vector<Weather> a = {Weather::Sunny, Weather::Cloudy, Weather::Cloudy,
      Weather::Rainy};
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
  for (Weather today : m->domain()) {
    std::cout << "If today is " << today << " yesterday was: ";
    std::cout << Weather::Sunny << "="
        << reverse->ConditionalProbabilityOf(Weather::Sunny, today) << ", ";
    std::cout << Weather::Cloudy << "="
        << reverse->ConditionalProbabilityOf(Weather::Cloudy, today) << ", ";
    std::cout << Weather::Rainy << "="
        << reverse->ConditionalProbabilityOf(Weather::Rainy, today)
        << std::endl;
  }
}
}  // namespace examples

