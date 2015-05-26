/*
 * ObservableWeatherBayes.cc
 *
 *  Based on exercise 2.3 from "Probabilistic Robotics" by Fox, Thrun,
 *  and Burgard.
 *
 *  Copyright (c) 2015, Norman Alan Oursland
 *  All rights reserved.
 */

#include "cognitoware/math/probability/discrete/ConditionalValueMap.h"
#include "cognitoware/math/probability/discrete/DistributionValueMap.h"
#include "cognitoware/math/probability/discrete/MarkovChain.h"
#include "cognitoware/math/probability/RandomConditional.h"
#include "cognitoware/math/probability/RandomDistribution.h"
#include "examples/Weather.h"
#include "gtest/gtest.h"

#include <iostream>
#include <memory>
#include <random>
#include <vector>
#include <utility>

using ::cognitoware::math::probability::discrete::ConditionalValueMap;
using ::cognitoware::math::probability::discrete::DistributionValueMap;
using ::cognitoware::math::probability::discrete::MarkovChain;
using ::cognitoware::math::probability::RandomConditional;
using ::cognitoware::math::probability::RandomDistribution;

namespace examples {

enum class WeatherObservation {
  Sunny, Cloudy, Rainy
};

std::ostream& operator<<(std::ostream& os, WeatherObservation z) {
  switch (z) {
  case WeatherObservation::Sunny:
    os << "Sunny";
  break;
  case WeatherObservation::Cloudy:
    os << "Cloudy";
  break;
  case WeatherObservation::Rainy:
    os << "Rainy";
  break;
  }
  return os;
}

std::shared_ptr<ConditionalValueMap<WeatherObservation, Weather>> CreateSensorModel() {
  auto result = std::make_shared<
      ConditionalValueMap<WeatherObservation, Weather>>();
  result->Set(WeatherObservation::Sunny, Weather::Sunny, 0.6);
  result->Set(WeatherObservation::Sunny, Weather::Cloudy, 0.3);
  result->Set(WeatherObservation::Sunny, Weather::Rainy, 0.0);
  result->Set(WeatherObservation::Cloudy, Weather::Sunny, 0.4);
  result->Set(WeatherObservation::Cloudy, Weather::Cloudy, 0.7);
  result->Set(WeatherObservation::Cloudy, Weather::Rainy, 0.0);
  result->Set(WeatherObservation::Rainy, Weather::Sunny, 0.0);
  result->Set(WeatherObservation::Rainy, Weather::Cloudy, 0.0);
  result->Set(WeatherObservation::Rainy, Weather::Rainy, 1.0);
  return result;
}

std::shared_ptr<DistributionValueMap<Weather>> CreateSunnyOutlook() {
  auto belief = std::make_shared<DistributionValueMap<Weather>>();
  belief->Set(Weather::Sunny, 1.0);
  return belief;
}

TEST(ObservableWeatherBayes, BayesFilter) {
  auto sensorModel = CreateSensorModel();
  auto systemModel = CreateWeatherMachine();
  auto x = CreateSunnyOutlook();
  std::vector<WeatherObservation> observationStream = {
      WeatherObservation::Cloudy, WeatherObservation::Cloudy,
      WeatherObservation::Rainy, WeatherObservation::Sunny};
  for (auto& z : observationStream) {
    x = systemModel->Marginalize(*x);  // get the expected weather change
    std::cout << "Wait: " << *x << std::endl;
    x = sensorModel->BayesianInference(z, *x);  // update with our x
    std::cout << "See: " << *x << std::endl;
    std::cout << std::endl;
  }
}

TEST(ObservableWeatherBayes, BayesFilter2) {
  auto sensorModel = CreateSensorModel();
  auto systemModel = CreateWeatherMachine();
  auto x = CreateSunnyOutlook();
  std::vector<WeatherObservation> observationStream = {
      WeatherObservation::Sunny, WeatherObservation::Sunny,
      WeatherObservation::Rainy};
  for (auto& z : observationStream) {
    x = systemModel->Marginalize(*x);  // get the expected weather change
    std::cout << "Wait: " << *x << std::endl;
    x = sensorModel->BayesianInference(z, *x);  // update with our x
    std::cout << "See: " << *x << std::endl;
    std::cout << std::endl;
  }
}

TEST(ObservableWeatherBayes, KnowledgeOfPast) {
  auto sensorModel = CreateSensorModel();  // p(z0|x)
  auto systemModel = CreateWeatherMachine();  // p(x1|x)

  std::vector<WeatherObservation> observationStream = {
      WeatherObservation::Sunny, WeatherObservation::Sunny,
      WeatherObservation::Rainy};
  for (unsigned int i = 0; i < observationStream.size(); i++) {
    auto x = CreateSunnyOutlook();
    // p(x_1') = [p(x_1|x_0) * p(x_0)] -- marginalize system model
    // p(x_1) = n * p(z_1|result) @ p(x_1') -- bayes sensor model
    // p(x_1) = n * p(z_1|result) @ [p(x_1|x_0) * p(x_0)]
    for (unsigned int j = 0; j <= i; j++) {
      auto z = observationStream[j];
      x = systemModel->Marginalize(*x);  // get the expected weather change
      x = sensorModel->BayesianInference(z, *x);  // update with our x
    }
    std::cout << "Knowledge of day " << i << " on day " << i << ":"
        << std::endl;
    std::cout << "Wait: " << *x << std::endl;
    // we need p(z_1|x_0), p(z_2|x_0), p(z_3|x_0)
    // p(z1|x) = p(z1|x1) * p(x1|x)
    auto p_zt_x0 = sensorModel;
    for (unsigned int j = i + 1; j < observationStream.size(); j++) {
      auto z = observationStream[j];
      // p(Z|X) = p(Z|X)->Marginalize(P(X|X))
      p_zt_x0 = p_zt_x0->Marginalize(*systemModel);
      x = p_zt_x0->BayesianInference(z, *x);
      std::cout << "Knowledge of day " << i << " on day " << j << ":"
          << std::endl;
      std::cout << "See: " << *x << std::endl;
    }
    std::cout << std::endl;
  }
}

}  // namespace examples
