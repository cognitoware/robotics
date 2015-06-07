/*
 * GaussianActionModel.h
 *
 *  Copyright (c) 2015, Norman Alan Oursland
 *  All rights reserved.
 */

#ifndef COGNITOWARE_ROBOTICS_STATE_ESTIMATION_GAUSSIANACTIONMODEL_H_
#define COGNITOWARE_ROBOTICS_STATE_ESTIMATION_GAUSSIANACTIONMODEL_H_

#include "cognitoware/math/data/Matrix.h"
#include "cognitoware/robotics/state_estimation/ActionModel.h"

namespace cognitoware {
namespace robotics {
namespace state_estimation {

template<typename U, typename X>
class GaussianActionModel : public ActionModel<U, X> {
public:
  GaussianActionModel() {
  }
  virtual ~GaussianActionModel() {}

  // Expected next state.
  virtual X GetMean(const U& action, const X& state) const = 0;
  // Process noise.
  virtual math::data::Matrix GetError(const U& action, const X& mean) const = 0;
};

}  // namespace state_estimation
}  // namespace robotics
}  // namespace cognitoware

#endif /* COGNITOWARE_ROBOTICS_STATE_ESTIMATION_GAUSSIANACTIONMODEL_H_ */
