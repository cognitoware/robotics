/*
 * DiscreteConditional.h
 *
 *  Copyright (c) 2015, Norman Alan Oursland
 *  All rights reserved.
 */

#ifndef MATH_PROBABILITY_DISCRETE_DISCRETECONDITIONAL_H_
#define MATH_PROBABILITY_DISCRETE_DISCRETECONDITIONAL_H_

#include "cognitoware/math/probability/RandomConditional.h"

#include <iterator>
#include <memory>

namespace cognitoware {
namespace math {
namespace probability {
namespace discrete {

template<typename X, typename Y>
class DiscreteConditional : public RandomConditional<X, Y> {
public:
  DiscreteConditional() {
  }

  ~DiscreteConditional() override {
  }
};

}  // namespace discrete
}  // namespace probability
}  // namespace math
}  // namespace cognitoware

#endif /* MATH_PROBABILITY_DISCRETE_DISCRETECONDITIONAL_H_ */
