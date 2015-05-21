/*
 * ConditionalMap.h
 *
 *  Copyright (c) 2015, Norman Alan Oursland
 *  All rights reserved.
 */

#ifndef COGNITOWARE_COGNITOWARE_MATH_PROBABILITY_CONDITIONALMAP_H_
#define COGNITOWARE_COGNITOWARE_MATH_PROBABILITY_CONDITIONALMAP_H_

#include "cognitoware/math/probability/RandomConditional.h"
#include "cognitoware/math/probability/RandomDistribution.h"
#include "cognitoware/math/probability/BayesDistribution.h"

#include <map>
#include <memory>
#include <type_traits>
#include <utility>
#include <map>

namespace cognitoware {
namespace math {
namespace probability {

template<typename X, typename Y>
class ConditionalMap : public RandomConditional<X, Y> {
public:
  ConditionalMap() {}
  virtual ~ConditionalMap() {}
  double ConditionalProbabilityOf(X x, Y y) const override {
    // TODO: Handle case for no y.
    return map_.at(y)->ProbabilityOf(x);
  }
  std::shared_ptr<RandomDistribution<Y>> BayesianInference(
      X observation, std::shared_ptr<const RandomDistribution<Y>> belief) const
          override {
    auto result = new BayesDistribution<Y, X>(belief, this->shared_from_this(),
        observation);
    return std::shared_ptr<RandomDistribution<Y>>(result);
  }
  void Set(Y y, std::shared_ptr<RandomDistribution<X>> x) {
    map_[y] = x;
  }

private:
  std::map<Y, std::shared_ptr<RandomDistribution<X>>> map_;
};

}  // namespace probability
}  // namespace math
}  // namespace cognitoware

#endif /* COGNITOWARE_COGNITOWARE_MATH_PROBABILITY_CONDITIONALMAP_H_ */
