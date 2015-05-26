/*
 * Weather.h
 *
 *  Copyright (c) 2015, Norman Alan Oursland
 *  All rights reserved.
 */

#ifndef WEATHER_H_
#define WEATHER_H_

#include "cognitoware/math/probability/RandomDistribution.h"
#include "cognitoware/math/probability/discrete/MarkovChain.h"

#include <iostream>

namespace examples {
using ::cognitoware::math::probability::RandomDistribution;
using ::cognitoware::math::probability::discrete::MarkovChain;

enum class Weather {
  Sunny, Cloudy, Rainy
};

std::ostream& operator<<(std::ostream& os, Weather w);
std::ostream& operator<<(std::ostream& os,
                         const RandomDistribution<Weather>& belief);
std::shared_ptr<MarkovChain<Weather>> CreateWeatherMachine();

}  // namespace examples

#endif /* WEATHER_H_ */
