/*
 * DiscreteDistribution.h
 *
 *  Copyright (c) 2015, Norman Alan Oursland
 *  All rights reserved.
 */

#ifndef COGNITOWARE_MATH_PROBABILITY_DISCRETE_DISCRETEDISTRIBUTION_H_
#define COGNITOWARE_MATH_PROBABILITY_DISCRETE_DISCRETEDISTRIBUTION_H_

#include "cognitoware/math/probability/RandomDistribution.h"

#include <map>

namespace cognitoware {
namespace math {
namespace probability {
namespace discrete {

template<typename X>
class DiscreteDistribution : public RandomDistribution<X> {
public:
  DiscreteDistribution() {
  }

  virtual ~DiscreteDistribution() {
  }

protected:
  template<typename NumberType>
  static X SelectFromMap(const std::map<X, NumberType>& map, double select) {
    if (map.empty()) {
      throw std::runtime_error("Cannot sample from an empty distribution");
    }
    X result;
    auto p = map.begin();
    do {
      result = p->first;
      select -= p->second;
    } while (++p != map.end() && select >= 0);
    return result;
  }
};

}  // namespace discrete
}  // namespace probability
}  // namespace math
}  // namespace cognitoware

#endif /* COGNITOWARE_MATH_PROBABILITY_DISCRETE_DISCRETEDISTRIBUTION_H_ */
