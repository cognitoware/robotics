/*
 * GaussianCanonical.h
 *
 *  Copyright (c) 2015, Norman Alan Oursland
 *  All rights reserved.
 */

#ifndef MATH_PROBABILITY_CONTINUOUS_GAUSSIANCANONICAL_H_
#define MATH_PROBABILITY_CONTINUOUS_GAUSSIANCANONICAL_H_

#include "cognitoware/math/data/Matrix.h"
#include "cognitoware/math/data/MatrixVectorOperators.h"
#include "cognitoware/math/data/Vector.h"
#include "cognitoware/math/probability/RandomDistribution.h"
#include "cognitoware/math/probability/continuous/GaussianMoment.h"

#include <math.h>
#include <memory>
#include <iostream>

namespace cognitoware {
namespace math {
namespace probability {
namespace continuous {

template<typename X>
class GaussianCanonical : public RandomDistribution<X> {
public:

  GaussianCanonical(const data::Vector& xi, const data::Matrix& omega) :
      information_(std::move(xi)),
          precision_(std::move(omega)),
          sigma_(omega.rows(), omega.cols()),
          mu_() {
  }
  explicit GaussianCanonical(const GaussianMoment<X>& x) :
      information_(x.covariance().Inverse()),
          precision_(information_ * x.mean()),
          mu_(x.mean()),
          sigma_(x.covariance()),
          is_sigma_valid_(true),
          is_mu_valid_(true) {
  }
  GaussianCanonical(GaussianCanonical<X> && that) :
      precision_(std::move(that.precision_)),
      information_(std::move(that.information_)),
      is_sigma_valid_(that.is_sigma_valid_),
      is_mu_valid_(that.is_mu_valid_),
      sigma_(std::move(that.sigma_)),
      mu_(std::move(that.mu_)) {
  }

  GaussianMoment<X>& operator=(GaussianMoment<X>&& that) {
      this->precision_ = std::move(that.precision_);
      this->information_ = std::move(that.information_);
      this->is_sigma_valid_ = that.is_sigma_valid_;
      this->is_mu_valid_ = that.is_mu_valid_;
      this->sigma_ = std::move(that.sigma_);
      this->mu_ = std::move(that.mu_);
      return *this;
  }

  ~GaussianCanonical() override {
  }

  const data::Vector& information() const {
    return information_;
  }
  const data::Matrix& precision() const {
    return precision_;
  }
  const data::Matrix& sigma() const {
    return InversePrecision();
  }
  const data::Vector& mu() const {
    return GetMu();
  }

  const data::Matrix& InversePrecision() const {
    if (!is_sigma_valid_) {
      sigma_ = precision().Inverse();
      is_sigma_valid_ = true;
    }
    return sigma_;
  }

  const X& GetMu() const {
    if (!is_mu_valid_) {
      mu_.Set(sigma() * information());
      is_mu_valid_ = true;
    }
    return mu_;
  }

  double ProbabilityOf(const X&) const override {
    throw std::runtime_error(
        "GaussianCanonical::ProbabilityOf not implemented.");
  }

  X Sample(std::default_random_engine*) const override {
    throw std::runtime_error("GaussianCanonical::Sample not implemented.");
  }

private:
  data::Vector information_;  // Omega
  data::Matrix precision_;  // xi
  mutable bool is_sigma_valid_ = false;
  mutable bool is_mu_valid_ = false;
  mutable data::Matrix sigma_;
  mutable X mu_;

};

template<typename X>
static std::ostream& operator<<(std::ostream& os,
                                const GaussianCanonical<X>& x) {
  os << "omega=" << x.information() << ", xi=" << x.precision();
  return os;
}

}  // namespace continuous
}  // namespace probability
}  // namespace math
}  // namespace cognitoware

#endif /* MATH_PROBABILITY_CONTINUOUS_GAUSSIANCANONICAL_H_ */
