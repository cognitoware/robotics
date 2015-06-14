/*
 * InformationFilter.h
 *
 *  Copyright (c) 2015, Norman Alan Oursland
 *  All rights reserved.
 */

#ifndef ROBOTICS_STATE_ESTIMATION_INFORMATIONFILTER_H_
#define ROBOTICS_STATE_ESTIMATION_INFORMATIONFILTER_H_

#include "cognitoware/robotics/state_estimation/KalmanFilter.h"
#include "cognitoware/math/probability/continuous/GaussianCanonical.h"

namespace cognitoware {
namespace robotics {
namespace state_estimation {

template<typename X, typename U, typename Z>
class InformationFilter {
public:
  InformationFilter() {}
  virtual ~InformationFilter() {}

  std::shared_ptr<math::probability::continuous::GaussianCanonical<X>>
  Marginalize(
      const KalmanActionModel<U, X>& model, const U& action,
      const math::probability::continuous::GaussianCanonical<X>& belief) {
    using math::data::Matrix;
    using math::data::Vector;
    using math::probability::continuous::GaussianCanonical;

    const Vector& xi = belief.information();
    const Matrix& A = model.a();
    const Matrix& B = model.b();
    const Vector& C = model.c();
    const Matrix& R = model.r();
    Matrix At = A.Transpose();
    Matrix Sigma = belief.InversePrecision();
    Matrix ASigma = A * Sigma;

    Matrix nextOmega = (ASigma * At + R).Inverse();
    X nextXi;
    nextXi.Set(nextOmega * (ASigma * xi + B * action + C));

    return std::make_shared<GaussianCanonical<X>>(std::move(nextXi), std::move(nextOmega));
  }

  std::shared_ptr<math::probability::continuous::GaussianCanonical<X>>
  BayesianInference(
      const KalmanSensorModel<Z, X>& model,
      const Z& data,
      const math::probability::continuous::GaussianCanonical<X>& belief) {
    using math::data::Matrix;
    using math::data::Vector;
    using math::probability::continuous::GaussianCanonical;

    const Vector& Xi = belief.information();
    const Matrix& Omega = belief.precision();
    const Matrix& C = model.c();
    Matrix Ct = C.Transpose();
    const Matrix Q = model.q();
    const Matrix QInv = Q.Inverse();
    const Matrix CtQInv = Ct * QInv;

    Matrix nextOmega = CtQInv * C + Omega;
    Vector nextXi = CtQInv * data + Xi;

    return std::make_shared<GaussianCanonical<X>>(std::move(nextXi), std::move(nextOmega));
  }
};

}  // namespace state_estimation
}  // namespace robotics
}  // namespace cognitoware

#endif /* ROBOTICS_STATE_ESTIMATION_INFORMATIONFILTER_H_ */
