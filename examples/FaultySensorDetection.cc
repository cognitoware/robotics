/*
 * FaultySensorDetection.cc
 *
 *  Created on: Jan 17, 2015
 *      Author: Alan Oursland
 */

#include "cognitoware/math/data/Vector.h"
#include "cognitoware/math/probability/ConditionalMap.h"
#include "cognitoware/math/probability/RandomDistribution.h"
#include "cognitoware/math/probability/RangedUniform.h"
#include "cognitoware/math/probability/DistributionValueMap.h"
#include "gtest/gtest.h"

#include <iostream>
#include <random>
#include <utility>

using ::cognitoware::math::data::Vector;
using ::cognitoware::math::probability::ConditionalMap;
using ::cognitoware::math::probability::RandomDistribution;
using ::cognitoware::math::probability::RangedUniform;
using ::cognitoware::math::probability::DistributionValueMap;

namespace examples {

class SensorReading : public Vector<SensorReading, 1> {
public:
  SensorReading()
      : Vector( { 0.0 }) {
  }
  SensorReading(double d)
      : Vector( { d }) {
  }
  SensorReading(const SensorReading& that)
      : Vector<SensorReading, 1>(that) {
  }
  SensorReading(SensorReading&& that)
      : Vector<SensorReading, 1>(that) {
  }
};

enum class SensorState {
  Normal, Faulty
};

class Sensor {
public:
  Sensor(SensorState state)
      : state_(state) {
  }
  SensorReading CreateObservation(std::default_random_engine* generator) {
    std::uniform_real_distribution<double> dist(0, 1);
    if (state_ == SensorState::Faulty) {
      return SensorReading(dist(*generator));
    } else {
      return SensorReading(3.0 * dist(*generator));
    }
  }
private:
  SensorState state_;
};

class SensorModel : public ConditionalMap<SensorReading, SensorState> {
public:
  SensorModel() {
    Set(SensorState::Normal,
        std::make_shared<RangedUniform<SensorReading>>(
            SensorReading(0.0),
            SensorReading(3.0)));
    Set(SensorState::Faulty,
        std::make_shared<RangedUniform<SensorReading>>(
            SensorReading(0.0),
            SensorReading(1.0)));
  }
};

TEST(FaultySensorDetection, main) {
  auto sensorModel = std::make_shared<SensorModel>();
  EXPECT_EQ(1.0 / 3.0,
      sensorModel->ConditionalProbabilityOf(SensorReading(0.5),
          SensorState::Normal));
  EXPECT_EQ(1.0,
      sensorModel->ConditionalProbabilityOf(SensorReading(0.5),
          SensorState::Faulty));

  std::default_random_engine generator(0);
  Sensor sensor(SensorState::Faulty);
  std::shared_ptr<const RandomDistribution<SensorState>> belief =
      std::make_shared<DistributionValueMap<SensorState>>(SensorState::Normal,
          0.99, SensorState::Faulty, 0.01);
  double lastNormal = belief->ProbabilityOf(SensorState::Normal);
  EXPECT_EQ(0.99, lastNormal);
  std::cout << lastNormal << std::endl;
  for (int i = 0; i < 25; i++) {
    SensorReading observation = sensor.CreateObservation(&generator);
    std::cout << observation.at(0) << std::endl;
    belief = sensorModel->BayesianInference(std::move(observation), belief);
    std::cout << belief->ProbabilityOf(SensorState::Normal) << std::endl;
  }

}

}  // namespace examples
