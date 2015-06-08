/*
 * KalmanFilter.h
 *
 *  Copyright (c) 2015, Norman Alan Oursland
 *  All rights reserved.
 */

#ifndef COGNITOWARE_ROBOTICS_STATE_ESTIMATION_KALMANFILTER_H_
#define COGNITOWARE_ROBOTICS_STATE_ESTIMATION_KALMANFILTER_H_

#include "cognitoware/math/data/Matrix.h"
#include "cognitoware/math/probability/continuous/GaussianMoment.h"
#include "cognitoware/robotics/state_estimation/GaussianActionModel.h"
#include "cognitoware/robotics/state_estimation/GaussianSensorModel.h"

#include <memory>
#include <stdexcept>

namespace cognitoware {
namespace robotics {
namespace state_estimation {

template<typename U, typename X>
class KalmanActionModel : public GaussianActionModel<U, X> {
public:
  KalmanActionModel() {
  }

  ~KalmanActionModel() override {
  }

  X GetMean(const U& action, const X& state) const override {
    X result;
    result.Set((a() * state) + (b() * action) + c());
    return result;
  }

  math::data::Matrix GetError(const U&, const X&) const override {
    return math::data::Matrix(r());
  }

  const math::data::Matrix& a() const {
    // n x n
    return a_;
  }
  const math::data::Matrix& b() const {
    // n x m
    return b_;
  }
  const math::data::Vector& c() const {
    // n
    return c_;
  }
  const math::data::Matrix& r() const {
    return r_;
  }

  math::data::Matrix& a() {
    return a_;
  }
  math::data::Matrix& b() {
    return b_;
  }
  math::data::Vector& c() {
    return c_;
  }
  math::data::Matrix& r() {
    return r_;
  }

private:
  math::data::Matrix a_;
  math::data::Matrix b_;
  math::data::Vector c_;
  math::data::Matrix r_;
};

template<typename Z, typename X>
class KalmanSensorModel : public GaussianSensorModel<Z, X>{
public:
  KalmanSensorModel() {}
  ~KalmanSensorModel() override {}

  double ConditionalProbabilityOf(const Z& observation, const X& state) const override {
    // TODO: Gaussian moment caches a bunch of stuff. I should store one here
    // instead of generating and solving one each time.
    return math::probability::continuous::GaussianMoment<Z>(
        GetMean(state), GetError(observation)).ProbabilityOf(observation);
  }

  Z GetMean(const X& state) const override {
    Z z;
    z.Set(c_ * state);
    return z;
  }

  math::data::Matrix GetError(const Z& ) const override {
    return q_;
  }

  const math::data::Matrix& c() const {
    // n x n
    return c_;
  }
  const math::data::Matrix& q() const {
    // n x n
    return q_;
  }
  math::data::Matrix& c() {
    return c_;
  }
  math::data::Matrix& q() {
    return q_;
  }

private:
  math::data::Matrix c_;
  math::data::Matrix q_;
};

template<typename X, typename U, typename Z>
class KalmanFilter {
public:
  KalmanFilter() {}

  virtual ~KalmanFilter() {}

  std::shared_ptr<math::probability::continuous::GaussianMoment<X>>
  BayesianInference(
      const KalmanSensorModel<Z, X>& model,
      const Z& data,
      const math::probability::continuous::GaussianMoment<X>& belief) const {
    using math::data::Matrix;
    using math::probability::continuous::GaussianMoment;
    // Table 3.1 p42
    auto& z = data;
    auto& mu0 = belief.mean();
    auto& sigma0 = belief.covariance();

    auto z_est = model.GetMean(mu0);
    auto Q = model.GetError(z);

    Matrix C = model.c();
    Matrix K;
    {
      Matrix sc = sigma0 * C.Transpose();
      K = sc * ((C * sc) + Q).Inverse();
    }

    X mu1;
    mu1.Set(mu0 + K * (z - z_est));

    Matrix I = Matrix::Identity(sigma0.rows());
    Matrix sigma1 = (I - K * C) * sigma0;

    return std::make_shared<GaussianMoment<X>>(mu1, sigma1);
  }

  std::shared_ptr<math::probability::continuous::GaussianMoment<X>>
  Marginalize(const KalmanActionModel<U, X>& model,
              const U& action,
              const math::probability::continuous::GaussianMoment<X>& state) const {
    using math::data::Matrix;
    using math::probability::continuous::GaussianMoment;
    // Table 3.1 p42
    const X& x1 = model.GetMean(action, state.mean()); // 2
    Matrix q = model.a() * state.covariance() * model.a().Transpose(); //2a
    q += model.GetError(action, state.mean()); // 2b
    return std::make_shared<GaussianMoment<X>>(
        std::move(x1),
        std::move(q));
  }

  std::shared_ptr<math::probability::continuous::GaussianMoment<X>>
  Marginalize(const KalmanActionModel<U, X>& model,
              const math::probability::continuous::GaussianMoment<U>& action,
              const math::probability::continuous::GaussianMoment<X>& state) const {
    using math::data::Matrix;
    using math::probability::continuous::GaussianMoment;
    auto x1 = Marginalize(model, action.mean(), state);
    const Matrix& A = model.a();
    const Matrix& B = model.b();
    Matrix q = A * state.covariance() * A.Transpose();
    q += (B * action.covariance() * B.Transpose());
    q += x1->covariance();
    return std::make_shared<GaussianMoment<X>>(x1->mean(), std::move(q));
  }

};

}  // namespace state_estimation
}  // namespace robotics
}  // namespace cognitoware

#endif /* COGNITOWARE_ROBOTICS_STATE_ESTIMATION_KALMANFILTER_H_ */
