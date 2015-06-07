/*
 * GaussianSensorModel.h
 *
 *  Copyright (c) 2015, Norman Alan Oursland
 *  All rights reserved.
 */

#ifndef ROBOTICS_STATE_ESTIMATION_GAUSSIANSENSORMODEL_H_
#define ROBOTICS_STATE_ESTIMATION_GAUSSIANSENSORMODEL_H_

#include "cognitoware/math/data/Matrix.h"
#include "cognitoware/robotics/state_estimation/SensorModel.h"

namespace cognitoware {
namespace robotics {
namespace state_estimation {

template<typename Z, typename X>
class GaussianSensorModel : public SensorModel<Z, X> {
public:
  GaussianSensorModel() {}
  virtual ~GaussianSensorModel() override {}

  virtual Z GetMean(const X& state) const = 0;
  virtual math::data::Matrix GetError(const Z& observation) const = 0;
};

}  // namespace state_estimation
}  // namespace robotics
}  // namespace cognitoware

#endif /* ROBOTICS_STATE_ESTIMATION_GAUSSIANSENSORMODEL_H_ */
