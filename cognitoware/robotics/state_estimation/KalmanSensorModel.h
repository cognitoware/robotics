/*
 * KalmanSensorModel.h
 *
 *  Copyright (c) 2015, Norman Alan Oursland
 *  All rights reserved.
 */

#ifndef ROBOTICS_STATE_ESTIMATION_KALMANSENSORMODEL_H_
#define ROBOTICS_STATE_ESTIMATION_KALMANSENSORMODEL_H_

#include "cognitoware/math/data/Matrix.h"
#include "cognitoware/math/data/Vector.h"
#include "cognitoware/math/data/MatrixVectorOperators.h"
#include "cognitoware/robotics/state_estimation/GaussianSensorModel.h"
#include "cognitoware/math/probability/continuous/GaussianMoment.h"

namespace cognitoware {
namespace robotics {
namespace state_estimation {

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

}  // namespace state_estimation
}  // namespace robotics
}  // namespace cognitoware

#endif /* ROBOTICS_STATE_ESTIMATION_KALMANSENSORMODEL_H_ */
