/*
 * SensorModel.h
 *
 *  Copyright (c) 2015, Norman Alan Oursland
 *  All rights reserved.
 */

#ifndef ROBOTICS_STATE_ESTIMATION_SENSORMODEL_H_
#define ROBOTICS_STATE_ESTIMATION_SENSORMODEL_H_

namespace cognitoware {
namespace robotics {
namespace state_estimation {

template<typename Z, typename X>
class SensorModel {
public:
  SensorModel() {}
  virtual ~SensorModel() {}

  virtual double ConditionalProbabilityOf(const Z& observation, const X& state) const = 0;
};

}  // namespace state_estimation
}  // namespace robotics
}  // namespace cognitoware

#endif /* ROBOTICS_STATE_ESTIMATION_SENSORMODEL_H_ */
