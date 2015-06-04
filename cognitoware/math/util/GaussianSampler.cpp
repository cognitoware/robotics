/*
 * GaussianSampler.cpp
 *
 *  Copyright (c) 2015, Norman Alan Oursland
 *  All rights reserved.
 */

#include "cognitoware/math/data/Vector.h"
#include "cognitoware/math/probability/continuous/GaussianMoment.h"
#include "cognitoware/math/util/GaussianSampler.h"

using ::cognitoware::math::data::Vector;
using ::cognitoware::math::probability::continuous::GaussianMoment;

namespace cognitoware {
namespace math {
namespace util {

/* static */
GaussianSampler GaussianSampler::gaussian_sampler_;

/* static */
const GaussianSampler& GaussianSampler::Singleton() {
  if (!gaussian_sampler_.IsInitialized()) {
    gaussian_sampler_.Init();
  }
  return gaussian_sampler_;
}

/* static */
double GaussianSampler::N01(double x) {
  return GaussianMoment<Vector>::Fn(x, 0, 1);
}

GaussianSampler::GaussianSampler() {
}

void GaussianSampler::Init() {
  is_initialized_ = true;
  data::PiecewiseLinearFunction pdf(N01, -38.8, 0.0, 388, 0.000000001,
      0.000000000001);
  cdf_ = pdf.PdfToCdf();
}

bool GaussianSampler::IsInitialized() {
  return is_initialized_;
}

double GaussianSampler::SampleN01(double f) const {
  if (f > 0.5) return -cdf_.Evaluate(2.0 - 2 * f);
  else return cdf_.Evaluate(2 * f);
}

}  // namespace util
}  // namespace math
}  // namespace cognitoware
