/*
 * ConditionalMap.h
 *
 *  Copyright (c) 2015, Norman Alan Oursland
 *  All rights reserved.
 */

#ifndef COGNITOWARE_MATH_PROBABILITY_CONDITIONALMAP_H_
#define COGNITOWARE_MATH_PROBABILITY_CONDITIONALMAP_H_

#include "cognitoware/math/probability/RandomConditional.h"
#include "cognitoware/math/probability/RandomDistribution.h"

#include <map>
#include <map>
#include <memory>
#include <type_traits>
#include <utility>

namespace cognitoware {
namespace math {
namespace probability {

template<typename X, typename Y>
class ConditionalMap : public RandomConditional<X, Y> {
public:
  ConditionalMap() {
  }

  virtual ~ConditionalMap() {
  }

  void Set(Y y, std::shared_ptr<RandomDistribution<X>> x) {
    map_[y] = x;
  }

  double ConditionalProbabilityOf(const X& x, const Y& y) const override {
    double result = 0.0;
    auto key_value = map_.find(y);
    if (key_value != map_.end()) {
      result = key_value->second->ProbabilityOf(x);
    }
    return result;
  }

private:
  std::map<Y, std::shared_ptr<RandomDistribution<X>>>map_;
};

} // namespace probability
} // namespace math
} // namespace cognitoware

#endif /* COGNITOWARE_MATH_PROBABILITY_CONDITIONALMAP_H_ */
