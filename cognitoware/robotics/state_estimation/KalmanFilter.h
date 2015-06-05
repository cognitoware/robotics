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
#include "cognitoware/robotics/state_estimation/KalmanActionModel.h"
#include "cognitoware/robotics/state_estimation/KalmanSensorModel.h"

#include <memory>
#include <stdexcept>

namespace cognitoware {
namespace robotics {
namespace state_estimation {

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
