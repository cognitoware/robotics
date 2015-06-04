/*
 * GaussianSampler.h
 *
 *  Copyright (c) 2015, Norman Alan Oursland
 *  All rights reserved.
 */

#ifndef MATH_UTIL_GAUSSIANSAMPLER_H_
#define MATH_UTIL_GAUSSIANSAMPLER_H_

#include "cognitoware/math/data/PiecewiseLinearFunction.h"

namespace cognitoware {
namespace math {
namespace util {

class GaussianSampler final {
public:
  static const GaussianSampler& Singleton();
  static double N01(double x);

  GaussianSampler();
  void Init();
  bool IsInitialized();
  double SampleN01(double f) const;

private:
  static GaussianSampler gaussian_sampler_;

  bool is_initialized_ = false;
  // Cumulative Distribution Function
  data::PiecewiseLinearFunction cdf_;
};

}  // namespace util
}  // namespace math
}  // namespace cognitoware

#endif /* MATH_UTIL_GAUSSIANSAMPLER_H_ */
