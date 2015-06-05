/*
 * DistributionValueMap.h
 *
 *  Copyright (c) 2015, Norman Alan Oursland
 *  All rights reserved.
 */

#ifndef COGNITOWARE_MATH_PROBABILITY_DISCRETE_DISTRIBUTIONVALUEMAP_H_
#define COGNITOWARE_MATH_PROBABILITY_DISCRETE_DISTRIBUTIONVALUEMAP_H_

#include "cognitoware/math/probability/discrete/DiscreteDistribution.h"

#include <cmath>
#include <map>
#include <memory>
#include <random>

namespace cognitoware {
namespace math {
namespace probability {
namespace discrete {

template<typename X>
class DistributionValueMap : public DiscreteDistribution<X> {
public:
  DistributionValueMap() {
  }

  DistributionValueMap(X x0, double p0, X x1, double p1) {
    Set(x0, p0);
    Set(x1, p1);
    Normalize();
  }

  DistributionValueMap(const std::vector<X>& list) {
    for (auto& x : list) {
      Set(x, 1.0);
    }
    Normalize();
  }

  ~DistributionValueMap() override {
  }

  double ProbabilityOf(const X& x) const override {
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
    return this->SelectFromMap(map_, select);
  }

  void Set(X x, double p) {
    map_[x] = p;
  }

  void Normalize() {
    double sum = 0.0;
    for (auto& pair : map_) {
      sum += pair.second;
    }
    for (auto& pair : map_) {
      pair.second /= sum;
    }
  }

  double GetEntropy() const {
    double result = 0.0;
    for (auto& pair : map_) {
      double p = pair.second;
      if (p != 0) {
        result -= p * log2(p);
      }
    }
    return result;
  }

  std::shared_ptr<DistributionValueMap<X>> Product(
      const DistributionValueMap<X>& that) const {
    auto result = std::make_shared<DistributionValueMap<X>>();
    for (auto& pair : map_) {
      double lp = pair.second;
      double rp = that.ProbabilityOf(pair.first);
      double p = lp * rp;
      if (p != 0) {
        result->Set(pair.first, p);
      }
    }
    result->Normalize();
    return result;
  }

private:
  std::map<X, double> map_;
};

}  // namespace discrete
}  // namespace probability
}  // namespace math
}  // namespace cognitoware

#endif /* COGNITOWARE_MATH_PROBABILITY_DISCRETE_DISTRIBUTIONVALUEMAP_H_ */
