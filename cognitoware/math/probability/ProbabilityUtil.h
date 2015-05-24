/*
 * ProbabilityUtil.h
 *
 *  Copyright (c) 2015, Norman Alan Oursland
 *  All rights reserved.
 */

#ifndef MATH_PROBABILITY_PROBABILITYUTIL_H_
#define MATH_PROBABILITY_PROBABILITYUTIL_H_

#include "cognitoware/math/probability/BayesDistribution.h"
#include "cognitoware/math/probability/RandomDistribution.h"

#include <memory>

namespace cognitoware {
namespace math {
namespace probability {

template<typename X, typename Y>
std::shared_ptr<RandomDistribution<Y>> BayesianInference(
    std::shared_ptr<const RandomDistribution<Y>> belief,
    std::shared_ptr<const RandomConditional<X, Y>> likelihood, X observation) {
  return std::make_shared<BayesDistribution<Y, X>>(belief, likelihood,
      observation);
}

}  // namespace probability
}  // namespace math
}  // namespace cognitoware

#endif /* MATH_PROBABILITY_PROBABILITYUTIL_H_ */
