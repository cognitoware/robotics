/*
 * Weather.cc
 *
 *  Copyright (c) 2015, Norman Alan Oursland
 *  All rights reserved.
 */

#include "examples/Weather.h"

#include <iostream>

namespace examples {

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

}  // namespace examples

