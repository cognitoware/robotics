/*
 * DiscreteConditional.h
 *
 *  Copyright (c) 2015, Norman Alan Oursland
 *  All rights reserved.
 */

#ifndef COGNITOWARE_MATH_PROBABILITY_DISCRETE_DISCRETECONDITIONAL_H_
#define COGNITOWARE_MATH_PROBABILITY_DISCRETE_DISCRETECONDITIONAL_H_

#include "cognitoware/math/probability/RandomConditional.h"

#include <iterator>
#include <memory>
#include <vector>

namespace cognitoware {
namespace math {
namespace probability {
namespace discrete {

template<typename X, typename Y>
class DiscreteConditional : public RandomConditional<X, Y> {
public:
  DiscreteConditional() {
  }

  virtual ~DiscreteConditional() override {
  }

  virtual std::vector<X> domain() const = 0;

  virtual std::vector<Y> range() const = 0;
};

}  // namespace discrete
}  // namespace probability
}  // namespace math
}  // namespace cognitoware

#endif /* COGNITOWARE_MATH_PROBABILITY_DISCRETE_DISCRETECONDITIONAL_H_ */
