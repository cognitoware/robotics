/*
 * RandomConditional.h
 *
 *  Copyright (c) 2015, Norman Alan Oursland
 *  All rights reserved.
 */

#ifndef COGNITOWARE_COGNITOWARE_MATH_PROBABILITY_RANDOMCONDITIONAL_H_
#define COGNITOWARE_COGNITOWARE_MATH_PROBABILITY_RANDOMCONDITIONAL_H_

#include "cognitoware/math/probability/RandomDistribution.h"

#include <memory>

namespace cognitoware {
namespace math {
namespace probability {

template<typename X, typename Y>
class RandomConditional : public std::enable_shared_from_this<
    RandomConditional<X, Y>> {
public:
  RandomConditional() {
  }

  virtual ~RandomConditional() {
  }

  virtual double ConditionalProbabilityOf(X x, Y y) const = 0;

};

}  // namespace probability
}  // namespace math
}  // namespace cognitoware

#endif /* COGNITOWARE_COGNITOWARE_MATH_PROBABILITY_RANDOMCONDITIONAL_H_ */
