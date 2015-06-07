/*
 * ExtendedKalmanSensorModel.h
 *
 *  Copyright (c) 2015, Norman Alan Oursland
 *  All rights reserved.
 */

#ifndef ROBOTICS_STATE_ESTIMATION_EXTENDEDKALMANSENSORMODEL_H_
#define ROBOTICS_STATE_ESTIMATION_EXTENDEDKALMANSENSORMODEL_H_

#include "cognitoware/math/data/Matrix.h"
#include "cognitoware/robotics/state_estimation/GaussianSensorModel.h"

namespace cognitoware {
namespace robotics {
namespace state_estimation {

template<typename Z, typename X>
class ExtendedKalmanSensorModel : public GaussianSensorModel<Z, X>{
public:
  ExtendedKalmanSensorModel() {}
  virtual ~ExtendedKalmanSensorModel() {}

  virtual math::data::Matrix GetJacobian(const X& x) const = 0;
};

}  // namespace state_estimation
}  // namespace robotics
}  // namespace cognitoware

#endif /* ROBOTICS_STATE_ESTIMATION_EXTENDEDKALMANSENSORMODEL_H_ */
