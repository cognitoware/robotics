/*
 * DistributionCounter.h
 *
 *  Copyright (c) 2015, Norman Alan Oursland
 *  All rights reserved.
 */

#ifndef COGNITOWARE_MATH_PROBABILITY_DISCRETE_DISTRIBUTIONCOUNTER_H_
#define COGNITOWARE_MATH_PROBABILITY_DISCRETE_DISTRIBUTIONCOUNTER_H_

#include "cognitoware/math/probability/discrete/DiscreteDistribution.h"

#include <map>

namespace cognitoware {
namespace math {
namespace probability {
namespace discrete {

template<typename X>
class DistributionCounter : public DiscreteDistribution<X> {
public:
  DistributionCounter() {
  }

  virtual ~DistributionCounter() {
  }

  double ProbabilityOf(const X& x) const override {
    double result = 0.0;
    auto key_value = map_.find(x);
    if (key_value != map_.end()) {
      double n = (double) key_value->second;
      result = n / total_;
    }
    return result;
  }

  X Sample(std::default_random_engine* generator) const override {
    std::uniform_real_distribution<double> random(0, total_);
    double select = random(*generator);
    return this->SelectFromMap(map_, select);
  }

  void AddObservation(X x) {
    int n = 0;
    auto key_value = map_.find(x);
    if (key_value != map_.end()) {
      n = key_value->second;
    }
    map_[x] = (n + 1);
    total_++;
  }

private:
  int total_ = 0;
  std::map<X, int> map_;
};

}  // namespace discrete
}  // namespace probability
}  // namespace math
}  // namespace cognitoware

#endif /* COGNITOWARE_MATH_PROBABILITY_DISCRETE_DISTRIBUTIONCOUNTER_H_ */
