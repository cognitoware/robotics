/*
 * DistributionValueMap.h
 *
 *  Copyright (c) 2015, Norman Alan Oursland
 *  All rights reserved.
 */

#ifndef MATH_PROBABILITY_DISTRIBUTIONVALUEMAP_H_
#define MATH_PROBABILITY_DISTRIBUTIONVALUEMAP_H_

#include "cognitoware/math/probability/RandomDistribution.h"

#include <map>
#include <random>

namespace cognitoware {
namespace math {
namespace probability {

template<typename X>
class DistributionValueMap : public RandomDistribution<X> {
public:
  DistributionValueMap(X x0, double p0, X x1, double p1) {
    Set(x0, p0);
    Set(x1, p1);
  }
  virtual ~DistributionValueMap() {}
  double ProbabilityOf(X x) const override {
    double result = 0.0;
    auto it = map_.find(x);
    if (it != map_.end()) {
      result = it->second;
    }
    return result;
  }
  X Sample(std::default_random_engine* generator) const override {
    std::uniform_real_distribution<double> random(0, 1);
    double select = random(*generator);
    X result;
    for (auto p : map_) {
      result = p.first;
      select -= p.second;
      if (select < 0) break;
    }
    return result;
  }

  void Set(X x, double p) {
    map_[x] = p;
  }

private:
  std::map<X, double> map_;
};

}  // namespace probability
}  // namespace math
}  // namespace cognitoware

#endif /* MATH_PROBABILITY_DISTRIBUTIONVALUEMAP_H_ */
