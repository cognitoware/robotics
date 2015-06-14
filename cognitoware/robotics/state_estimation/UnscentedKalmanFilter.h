/*
 * UnscentedKalmanFilter.h
 *
 *  Copyright (c) 2015, Norman Alan Oursland
 *  All rights reserved.
 */

#ifndef ROBOTICS_STATE_ESTIMATION_UNSCENTEDKALMANFILTER_H_
#define ROBOTICS_STATE_ESTIMATION_UNSCENTEDKALMANFILTER_H_

#include "cognitoware/math/data/Matrix.h"
#include "cognitoware/math/data/Vector.h"
#include "cognitoware/math/probability/continuous/GaussianMoment.h"
#include "cognitoware/robotics/state_estimation/GaussianActionModel.h"
#include "cognitoware/robotics/state_estimation/GaussianSensorModel.h"

#include <memory>
#include <vector>

namespace cognitoware {
namespace robotics {
namespace state_estimation {

template<typename X2, typename Y>
struct SigmaPoint {
  static std::vector<SigmaPoint<X2, Y>> CreateSigmaPoints(
      const math::probability::continuous::GaussianMoment<X2>& belief,
      double alpha, double beta, double lambda) {
    using math::data::Matrix;
    using math::data::Vector;

    std::size_t order = belief.order();
    double n = static_cast<double>(order);

    Matrix p0 = ((n + lambda) * belief.covariance());
    // we need to ensure that P0 is symmetric
    Matrix p1 = p0.RoundSymmetry();
    Matrix p = p1.Sqrt().Transpose();

    std::vector<SigmaPoint<X2, Y>> sigma_pts(2 * order + 1);

    double wm0 = lambda / (lambda + n);
    double wc0 = wm0 + (1 - (alpha * alpha) + beta);
    double wn = 1 / (2 * (lambda + n));

    sigma_pts[0] = SigmaPoint<X2, Y>(belief.mean(), wm0, wc0);
    for (std::size_t i = 0; i < order; i++) {
      X2 x = belief.mean();
      Vector dx = p.GetColumn(i);
      X2 x1;
      x1.Set(x + dx);
      X2 x2;
      x2.Set(x - dx);
      sigma_pts[i + 1] = SigmaPoint<X2, Y>(x1, wn, wn);
      sigma_pts[i + 1 + order] = SigmaPoint<X2, Y>(x2, wn, wn);
    }
    return sigma_pts;
  }

  static Y GetWeightedMean(const std::vector<SigmaPoint<X2, Y>>& sigma_pts,
                           std::size_t n) {
    using math::data::Vector;
    std::vector<double> a(n);
    for (std::size_t i = 0; i < sigma_pts.size(); i++) {
      SigmaPoint<X2, Y> sp = sigma_pts[i];
      for (std::size_t j = 0; j < a.size(); j++) {
        a[j] += sp.mean_weight * sp.post[j];
      }
    }
    Vector y(std::move(a));
    return Y(std::move(y));
  }

  static math::data::Matrix GetWeightedError(
      const std::vector<SigmaPoint<X2, Y>>& sigmaPts, std::size_t n, Y next) {
    using math::data::Matrix;
    using math::data::Vector;

    Matrix next_e(n, n);
    for (std::size_t i = 0; i < sigmaPts.size(); i++) {
      SigmaPoint<X2, Y> sp = sigmaPts[i];
      Vector dt = sp.post - next;
      next_e = next_e + (sp.error_weight * dt.Cross(dt) + sp.error);
    }
    return next_e;
  }

  X2 prior;
  Y post;
  math::data::Matrix error;
  double mean_weight;
  double error_weight;

  SigmaPoint(X2 p, double w1, double w2) :
      prior(p), mean_weight(w1), error_weight(w2) {
  }

  SigmaPoint() :
      prior(), mean_weight(0.0), error_weight(0.0) {
  }
};

template<typename X, typename U, typename Z>
class UnscentedKalmanFilter {
public:
  UnscentedKalmanFilter(double alpha, double beta, double kappa) :
      alpha_(alpha), beta_(beta), kappa_(kappa) {
  }
  virtual ~UnscentedKalmanFilter() {
  }

  ::std::shared_ptr<math::probability::continuous::GaussianMoment<X>> Marginalize(
      const GaussianActionModel<U, X>& model, const U& action,
      const math::probability::continuous::GaussianMoment<X>& belief) {
    using math::data::Matrix;
    using math::probability::continuous::GaussianMoment;

    std::vector<SigmaPoint<X, X>> sigma_x = SigmaPoint<X, X>::CreateSigmaPoints(
        belief, alpha_, beta_, lambda(belief));

    for (std::size_t i = 0; i < sigma_x.size(); i++) {
      sigma_x[i].post = model.GetMean(action, sigma_x[i].prior);
      sigma_x[i].error = model.GetError(action, belief.mean());
    }

    std::size_t n = belief.order();
    X next_x = SigmaPoint<X, X>::GetWeightedMean(sigma_x, n);
    Matrix next_e = SigmaPoint<X, X>::GetWeightedError(sigma_x, n, next_x);

    return std::make_shared<GaussianMoment<X>>(next_x, next_e);
  }

  ::std::shared_ptr<math::probability::continuous::GaussianMoment<X>> BayesianInference(
      const GaussianSensorModel<Z, X>& model, const Z& observation,
      const math::probability::continuous::GaussianMoment<X>& belief) {
    using math::data::Matrix;
    using math::data::Vector;
    using math::probability::continuous::GaussianMoment;

    std::vector<SigmaPoint<X, Z>> sigma_z = SigmaPoint<X, Z>::CreateSigmaPoints(
        belief, alpha_, beta_, lambda(belief));

    for (std::size_t i = 0; i < sigma_z.size(); i++) {
      sigma_z[i].post = model.GetMean(sigma_z[i].prior);
      sigma_z[i].error = model.GetError(sigma_z[i].post);
    }

    //TransformSigmaPoints<Z>(sigmaZ, SensorModel.GetExpectedObservation);
    std::size_t n = belief.order();
    Z next_z = SigmaPoint<X, Z>::GetWeightedMean(sigma_z, n);
    Matrix S = SigmaPoint<X, Z>::GetWeightedError(sigma_z, n, next_z);

    Matrix E(n, n);
    for (std::size_t i = 0; i < sigma_z.size(); i++) {
      SigmaPoint<X, Z> sp = sigma_z[i];
      Vector dx = sigma_z[i].prior - belief.mean();
      Vector du = sigma_z[i].post - next_z;
      E = E + sp.error_weight * dx.Cross(du);
    }

    Matrix K = E * S.Inverse();

    X next_x(belief.mean() + K * (observation - next_z));
    Matrix next_e = belief.covariance() - K * S * K.Transpose();

    return std::make_shared<GaussianMoment<X>>(next_x, next_e);
  }

private:
  double lambda(const math::probability::continuous::GaussianMoment<X>& belief) const {
    double a = alpha_;
    double k = kappa_;
    double n = static_cast<double>(belief.order());
    return (a * a) * (n + k) - n;
  }

  // primary scaling factor and determines the spread of the Sigma points
  // around the x mean and is usually set to a small positive value
  // ( e.g., 1e-3).
  double alpha_;

  // Beta is the secondary scaling factor and determines the weight given to
  // the mean when recombining the sigma point.
  // Beta incorporates additional higher order knowledge about the distribution.
  // Beta=2 indicates the distribution is Gaussian.
  double beta_;

  // Kappa is a tertiary scaling parameter and is usually set to zero.
  double kappa_;

};

}  // namespace state_estimation
}  // namespace robotics
}  // namespace cognitoware

#endif /* ROBOTICS_STATE_ESTIMATION_UNSCENTEDKALMANFILTER_H_ */
