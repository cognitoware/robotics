/*
 * KalmanActionModel.h
 *
 *  Copyright (c) 2015, Norman Alan Oursland
 *  All rights reserved.
 */

#ifndef COGNITOWARE_ROBOTICS_STATE_ESTIMATION_KALMANACTIONMODEL_H_
#define COGNITOWARE_ROBOTICS_STATE_ESTIMATION_KALMANACTIONMODEL_H_

#include "cognitoware/math/data/Matrix.h"
#include "cognitoware/math/data/Vector.h"
#include "cognitoware/math/data/MatrixVectorOperators.h"
#include "cognitoware/robotics/state_estimation/GaussianActionModel.h"
#include "cognitoware/math/probability/continuous/GaussianMoment.h"

#include <memory>

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

}  // namespace state_estimation
}  // namespace robotics
}  // namespace cognitoware

#endif /* COGNITOWARE_ROBOTICS_STATE_ESTIMATION_KALMANACTIONMODEL_H_ */
