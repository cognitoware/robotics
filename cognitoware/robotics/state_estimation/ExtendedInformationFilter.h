/*
 * ExtendedInformationFilter.h
 *
 *  Copyright (c) 2015, Norman Alan Oursland
 *  All rights reserved.
 */

#ifndef ROBOTICS_STATE_ESTIMATION_EXTENDEDINFORMATIONFILTER_H_
#define ROBOTICS_STATE_ESTIMATION_EXTENDEDINFORMATIONFILTER_H_

#include "cognitoware/robotics/state_estimation/ExtendedKalmanFilter.h"
#include "cognitoware/math/probability/continuous/GaussianCanonical.h"

namespace cognitoware {
namespace robotics {
namespace state_estimation {

template<typename X, typename U, typename Z>
class ExtendedInformationFilter {
public:
  ExtendedInformationFilter() {}
  virtual ~ExtendedInformationFilter() {}

  std::shared_ptr<math::probability::continuous::GaussianCanonical<X>>
  Marginalize(
      const ExtendedKalmanActionModel<U, X>& model,
      const U& action,
      const math::probability::continuous::GaussianCanonical<X>& belief) {
    using math::data::Matrix;
    using math::data::Vector;
    using math::probability::continuous::GaussianCanonical;

    const X& mu = belief.GetMu();
    const Matrix& Sigma = belief.InversePrecision();

    Matrix G = model.GetStateJacobian(action, mu);
    Matrix Gt = G.Transpose();
    Matrix R = model.GetError(action, mu).Inverse();
    Matrix nextOmega = (G * Sigma * Gt + R).Inverse();

    X nextXi;
    nextXi.Set(nextOmega * model.GetMean(action, mu));

    return std::make_shared<GaussianCanonical<X>>(nextXi, nextOmega);
  }

  std::shared_ptr<math::probability::continuous::GaussianCanonical<X>>
  BayesianInference(
      const ExtendedKalmanSensorModel<Z, X>& model,
      const Z& observation,
      const math::probability::continuous::GaussianCanonical<X>& belief) {
    using math::data::Matrix;
    using math::data::Vector;
    using math::probability::continuous::GaussianCanonical;

    const X& mu = belief.GetMu();
    const Matrix& Omega = belief.precision();

    Matrix H = model.GetJacobian(mu);
    Matrix Ht = H.Transpose();
    Matrix Q = model.GetError(observation);
    Matrix QInv = Q.Inverse();
    Matrix nextOmega = Omega + Ht * QInv * H;

    const Vector& xi = belief.information();
    Z expectedObservation = model.GetMean(mu);

    Vector nextXi = xi + Ht * QInv * (observation - expectedObservation + H * mu);

    return std::make_shared<GaussianCanonical<X>>(nextXi, nextOmega);
  }
};

}  // namespace state_estimation
}  // namespace robotics
}  // namespace cognitoware

#endif /* ROBOTICS_STATE_ESTIMATION_EXTENDEDINFORMATIONFILTER_H_ */
