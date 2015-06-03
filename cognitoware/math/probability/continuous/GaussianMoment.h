/*
 * GaussianMoment.h
 *
 *  Copyright (c) 2015, Norman Alan Oursland
 *  All rights reserved.
 */

#ifndef MATH_PROBABILITY_CONTINUOUS_GAUSSIANMOMENT_H_
#define MATH_PROBABILITY_CONTINUOUS_GAUSSIANMOMENT_H_

#include "cognitoware/math/data/Matrix.h"

#include <memory>
#include <iostream>

namespace cognitoware {
namespace math {
namespace probability {
namespace continuous {

template<typename X>
class GaussianMoment {
public:
  GaussianMoment(X mean,
                 data::Matrix covariance) :
      mean_(std::move(mean)), covariance_(std::move(covariance)) {
  }

  virtual ~GaussianMoment() {
  }

  const X& mean() const { return mean_; }
  const math::data::Matrix& covariance() const { return covariance_; }

private:
  X mean_;
  data::Matrix covariance_;
};

}  // namespace continuous
}  // namespace probability
}  // namespace math
}  // namespace cognitoware

#endif /* MATH_PROBABILITY_CONTINUOUS_GAUSSIANMOMENT_H_ */
