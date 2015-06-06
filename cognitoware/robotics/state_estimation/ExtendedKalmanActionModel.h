/*
 * ExtendedKalmanActionModel.h
 *
 *  Copyright (c) 2015, Norman Alan Oursland
 *  All rights reserved.
 */

#ifndef ROBOTICS_STATE_ESTIMATION_EXTENDEDKALMANACTIONMODEL_H_
#define ROBOTICS_STATE_ESTIMATION_EXTENDEDKALMANACTIONMODEL_H_

#include "cognitoware/math/data/Matrix.h"
#include "cognitoware/robotics/state_estimation/GaussianActionModel.h"

namespace cognitoware {
namespace robotics {
namespace state_estimation {

template<typename U, typename X>
class ExtendedKalmanActionModel : public GaussianActionModel<U, X> {
public:
  ExtendedKalmanActionModel() {}
  virtual ~ExtendedKalmanActionModel() {}

  virtual math::data::Matrix GetStateJacobian(const U& action,
                                              const X& state) const = 0;
  virtual math::data::Matrix GetActionJacobian(const U& action,
                                               const X& state) const = 0;
};

}  // namespace state_estimation
}  // namespace robotics
}  // namespace cognitoware

#endif /* ROBOTICS_STATE_ESTIMATION_EXTENDEDKALMANACTIONMODEL_H_ */
