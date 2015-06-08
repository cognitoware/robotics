/*
 * ExtendedKalmanFilter.h
 *
 *  Copyright (c) 2015, Norman Alan Oursland
 *  All rights reserved.
 */

#ifndef ROBOTICS_STATE_ESTIMATION_EXTENDEDKALMANFILTER_H_
#define ROBOTICS_STATE_ESTIMATION_EXTENDEDKALMANFILTER_H_

#include "cognitoware/math/data/Matrix.h"
#include "cognitoware/math/data/Vector.h"
#include "cognitoware/math/probability/continuous/GaussianMoment.h"
#include "cognitoware/robotics/state_estimation/GaussianActionModel.h"
#include "cognitoware/robotics/state_estimation/GaussianSensorModel.h"

#include <memory>

namespace cognitoware {
namespace robotics {
namespace state_estimation {

template<typename U, typename X>
class ExtendedKalmanActionModel : public GaussianActionModel<U, X> {
public:
  ExtendedKalmanActionModel() {}
  virtual ~ExtendedKalmanActionModel() {}

  virtual math::data::Matrix GetStateJacobian(const U& action,
                                              const X& state) const = 0;
  virtual math::data::Matrix GetActionJacobian(const U& action,
                                               const X& state) const = 0;
};

template<typename Z, typename X>
class ExtendedKalmanSensorModel : public GaussianSensorModel<Z, X>{
public:
  ExtendedKalmanSensorModel() {}
  virtual ~ExtendedKalmanSensorModel() {}

  virtual math::data::Matrix GetJacobian(const X& x) const = 0;
};

template<typename X, typename U, typename Z>
class ExtendedKalmanFilter {
public:
  ExtendedKalmanFilter() {}
  virtual ~ExtendedKalmanFilter() {}

  ::std::shared_ptr<math::probability::continuous::GaussianMoment<X>>
   Marginalize(
      const ExtendedKalmanActionModel<U, X>& model,
      const U& action,
      const math::probability::continuous::GaussianMoment<X>& belief) {
    using ::cognitoware::math::data::Matrix;
    using ::cognitoware::math::probability::continuous::GaussianMoment;

    X x0 = belief.mean();
    Matrix e0 = belief.covariance();

    // time update
    Matrix g = model.GetStateJacobian(action, x0);
    Matrix gt = g.Transpose();
    Matrix r = model.GetError(action, x0);

    X x1 = model.GetMean(action, x0);
    Matrix e1 = g * e0 * gt + r;

    return std::make_shared<GaussianMoment<X>>(x1, e1);
  }

  ::std::shared_ptr<math::probability::continuous::GaussianMoment<X>>
   Marginalize(
       const ExtendedKalmanActionModel<U, X>& model,
       const math::probability::continuous::GaussianMoment<U>& action,
       const math::probability::continuous::GaussianMoment<X>& belief) {
    using ::cognitoware::math::data::Matrix;
    using ::cognitoware::math::probability::continuous::GaussianMoment;

    X x0 = belief.mean();
    U u = action.mean();
    Matrix e0 = belief.covariance();
    Matrix eu = action.covariance();

    // time update
    Matrix gx = model.GetStateJacobian(u, x0);
    Matrix gxt = gx.Transpose();
    Matrix gu = model.GetActionJacobian(action.Mean, x0);
    Matrix gut = gu.Transpose();
    Matrix r = model.GetError(u, x0);

    X x1 = model.GetMean(u, x0);
    Matrix e1 = gx * e0 * gxt + gu * eu *gut + r;

    return std::make_shared<GaussianMoment<X>>(x1, e1);
  }


  ::std::shared_ptr<math::probability::continuous::GaussianMoment<X>>
   BayesianInference(
       const ExtendedKalmanSensorModel<Z, X>& sensor_model,
       const Z& observation,
       const math::probability::continuous::GaussianMoment<X>& belief) {
    using ::cognitoware::math::data::Matrix;
    using ::cognitoware::math::data::Vector;
    using ::cognitoware::math::probability::continuous::GaussianMoment;

    // Table 3.3 p59
    X x0 = belief.mean();
    Matrix e0 = belief.covariance();

    Z expected_z = sensor_model.GetMean(x0);
    Matrix q = sensor_model.GetError(observation);

    Matrix h = sensor_model.GetJacobian(x0);
    Matrix ht = h.Transpose();

    Matrix i = Matrix::Identity(e0.order());

    Matrix e0_ht = e0 * ht;
    Matrix k = e0_ht * (h * e0_ht + q).Inverse();
    Vector v = (x0 + k * (observation - expected_z));
    X x1;
    x1.Set(std::move(v));
    Matrix e1 = (i - k * h) * e0;

    return std::make_shared<GaussianMoment<X>>(x1, e1);
  }
};

}  // namespace state_estimation
}  // namespace robotics
}  // namespace cognitoware

#endif /* ROBOTICS_STATE_ESTIMATION_EXTENDEDKALMANFILTER_H_ */
