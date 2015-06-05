/*
 * ConditionalValueMap.h
 *
 *  Copyright (c) 2015, Norman Alan Oursland
 *  All rights reserved.
 */

#ifndef COGNITOWARE_MATH_PROBABILITY_DISCRETE_CONDITIONALVALUEMAP_H_
#define COGNITOWARE_MATH_PROBABILITY_DISCRETE_CONDITIONALVALUEMAP_H_

#include "cognitoware/math/probability/discrete/DiscreteConditional.h"
#include "cognitoware/math/probability/discrete/DistributionValueMap.h"

#include <map>
#include <memory>
#include <random>
#include <vector>

namespace cognitoware {
namespace math {
namespace probability {
namespace discrete {

template<typename X, typename Y>
class ConditionalValueMap : public DiscreteConditional<X, Y> {
public:
  ConditionalValueMap() {
  }

  ~ConditionalValueMap() override {
  }

  std::vector<X> domain() const override {
    std::vector<X> result;
    result.reserve(domain_.size());
    for (auto& pair : domain_) {
      result.push_back(pair.first);
    }
    return result;
  }

  std::vector<Y> range() const override {
    std::vector<Y> result;
    result.reserve(map_.size());
    for (auto& pair : map_) {
      result.push_back(pair.first);
    }
    return result;
  }

  double ConditionalProbabilityOf(const X& x, const Y& y) const override {
    auto y_map = map_.find(y);
    if (y_map == map_.end()) {
      // TODO: Does zero make sense here? Does an unknown imply zero chance?
      return 0.0;
    }
    auto x_value = y_map->second.find(x);
    if (x_value == y_map->second.end()) {
      return 0.0;
    }
    return x_value->second;
  }

  void Set(const X& x, const Y& y, double p) {
    auto value_map = map_.find(y);
    if (value_map == map_.end()) {
      value_map = map_.emplace(y, std::map<X, double>()).first;
    }
    value_map->second[x] = p;
    domain_[x] = x;
  }

  std::shared_ptr<DistributionValueMap<Y>> LikelihoodOf(const X& data) const {
    auto result = std::make_shared<DistributionValueMap<Y>>();
    for (auto& y_map : map_) {
      auto x_value = y_map.second.find(data);
      if (x_value != y_map.second.end()) {
        result->Set(y_map.first, x_value->second);
      }
    }
    return result;
  }

  std::shared_ptr<DistributionValueMap<Y>> BayesianInference(
      X data, const DistributionValueMap<Y>& prior) const {
    auto likelihood = LikelihoodOf(data);
    return likelihood->Product(prior);
  }

  double SumLikelihood(const X& x) const {
    double sum = 0.0;
    for (auto& y_map : map_) {
      auto x_value = y_map.second.find(x);
      if (x_value != y_map.second.end()) {
        sum += x_value->second;
      }
    }
    return sum;
  }

  Y SampleLikelihood(const X& x, std::default_random_engine* generator) const {
    double sum = SumLikelihood(x);
    if (sum == 0.0) {
      throw std::runtime_error(
          "Conditional distribution is not conditioned on x. Cannot sample.");
    }
    std::uniform_real_distribution<double> random(0, sum);
    return SampleLikelihood(x, random(*generator));
  }

  Y SampleLikelihood(const X& x, double p) const {
    for (auto& y_map : map_) {
      Y y = y_map.first;
      auto x_value = y_map.second.find(x);
      if (x_value != y_map.second.end()) {
        p -= x_value->second;
        if (p < 0) {
          return y;
        }
      }
    }
    throw std::runtime_error(
        "Conditional distribution is not conditioned on x. Cannot sample.");
  }

  template<typename Z>
  std::shared_ptr<ConditionalValueMap<X, Z>> Marginalize(
      const DiscreteConditional<Y, Z>& that) {
    auto result = std::make_shared<ConditionalValueMap<X, Z>>();
    for (X& x : this->domain()) {
      for (Z& z : that.range()) {
        double p = 0.0;
        for (Y& y : that.domain()) {
          double xy = this->ConditionalProbabilityOf(x, y);
          double yz = that.ConditionalProbabilityOf(y, z);
          p += xy * yz;
        }
        result->Set(x, z, p);
      }
    }
    return result;
  }

private:
  std::map<Y, std::map<X, double>> map_;
  std::map<X, X> domain_;

};

}  // namespace discrete
}  // namespace probability
}  // namespace math
}  // namespace cognitoware

#endif /* COGNITOWARE_MATH_PROBABILITY_DISCRETE_CONDITIONALVALUEMAP_H_ */
