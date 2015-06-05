/*
 * GaussianMoment.h
 *
 *  Copyright (c) 2015, Norman Alan Oursland
 *  All rights reserved.
 */

#ifndef MATH_PROBABILITY_CONTINUOUS_GAUSSIANMOMENT_H_
#define MATH_PROBABILITY_CONTINUOUS_GAUSSIANMOMENT_H_

#include "cognitoware/math/data/Matrix.h"
#include "cognitoware/math/data/MatrixVectorOperators.h"
#include "cognitoware/math/data/Vector.h"
#include "cognitoware/math/probability/RandomDistribution.h"

#include <math.h>
#include <memory>
#include <iostream>

namespace cognitoware {
namespace math {
namespace probability {
namespace continuous {

template<typename X>
class GaussianMoment : public RandomDistribution<X> {
public:
  static double Fn(double x, double mean, double variance) {
    double A = sqrt(1.0 / (2 * M_PI * variance));
    double dx = x - mean;
    double x2 = dx * dx;
    double e = exp(-0.5 * x2 / variance);
    return A * e;
  }
  static double Fn(data::Vector x, data::Vector mean, data::Matrix covariance,
                   data::Matrix inverse_covariance) {
    double A = sqrt(1.0 / ((2 * M_PI) * covariance).Determinant());
    data::Vector dx = x - mean;
    double e = exp(-(dx * inverse_covariance * dx));
    return A * e;
  }
  static double Fn(data::Vector x, data::Vector mean, data::Matrix covariance) {
    return Fn(x, mean, covariance, covariance.Inverse());
  }

  GaussianMoment(X mean, data::Matrix covariance) :
      mean_(std::move(mean)), covariance_(std::move(covariance)) {
  }

  virtual ~GaussianMoment() {
  }

  const X& mean() const {
    return mean_;
  }
  const math::data::Matrix& covariance() const {
    return covariance_;
  }
  const math::data::Matrix& InverseCovariance() const {
    if (!is_valid_inverse_) {
      inverse_covariance_ = covariance_.Inverse();
      is_valid_inverse_ = true;
    }
    return inverse_covariance_;
  }
  const math::data::Matrix& SqrtCovariance() const {
    if (!is_valid_sqrt_) {
      srqt_covariance_ = covariance().Sqrt();
      is_valid_sqrt_ = true;
    }
    return srqt_covariance_;
  }
  std::size_t order() const {
    return mean().order();
  }

  double ProbabilityOf(const X& x) const override {
    return Fn(x.AliasVector(), mean().AliasVector(), covariance(),
        InverseCovariance());
  }

  X Sample(std::default_random_engine* generator) const override {
    std::normal_distribution<double> n01(0, 1);
    data::Vector sample(order());
    for (std::size_t i = 0; i < sample.order(); i++) {
      // need a N(0,1) here
      sample[i] = n01(*generator);
    }
    X result;
    result.Set(mean() + SqrtCovariance() * sample);
    return result;
  }

private:
  X mean_;
  data::Matrix covariance_;
  // These values are mutable because they are cached derivations of covariance_
  mutable data::Matrix inverse_covariance_;
  mutable data::Matrix srqt_covariance_;
  mutable bool is_valid_inverse_ = false;
  mutable bool is_valid_sqrt_ = false;
};

}  // namespace continuous
}  // namespace probability
}  // namespace math
}  // namespace cognitoware

#endif /* MATH_PROBABILITY_CONTINUOUS_GAUSSIANMOMENT_H_ */
