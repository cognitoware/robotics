/*
 * MarkovActionChain.h
 *
 *  Copyright (c) 2015, Norman Alan Oursland
 *  All rights reserved.
 */

#ifndef ROBOTICS_MARKOVACTIONCHAIN_H_
#define ROBOTICS_MARKOVACTIONCHAIN_H_

#include "cognitoware/math/probability/discrete/DiscreteConditional.h"
#include "cognitoware/math/probability/discrete/DistributionValueMap.h"
#include "cognitoware/math/probability/RandomConditional.h"

#include <map>

namespace cognitoware {
namespace robotics {

template<typename X, typename U>
class MarkovActionChain : public math::probability::discrete::DiscreteConditional<
    X, std::pair<X, U>> {
public:
  MarkovActionChain() {
  }

  ~MarkovActionChain() override {
  }

  void Set(X end, U action, X start, double p) {
    auto start_action_map = p_.find(start);
    if (start_action_map == p_.end()) {
      start_action_map =
          p_.emplace(start, std::map<U, std::map<X, double>>()).first;
    }
    auto action_end_map = start_action_map->second.find(action);
    if (action_end_map == start_action_map->second.end()) {
      auto emplace_result = start_action_map->second.emplace(action,
          std::map<X, double>());
      action_end_map = emplace_result.first;
    }
    action_end_map->second[end] = p;
  }

  double ConditionalProbabilityOf(const X& x, const std::pair<X, U>& xu) const
      override {
    const X& start = xu.first;
    const U& u = xu.second;
    auto start_action_map = p_.find(start);
    if (start_action_map == p_.end()) {
      return 0.0;
    }
    auto action_end_map = start_action_map->second.find(u);
    if (action_end_map == start_action_map->second.end()) {
      return 0.0;
    }
    auto end_value = action_end_map->second.find(x);
    if (end_value == action_end_map->second.end()) {
      return 0.0;
    }
    return end_value->second;
  }

  X Sample(U action, X start, double p) {
    auto start_action_map = p_.find(start);
    if (start_action_map == p_.end()) {
      throw std::runtime_error(
          "MarkovActionChain does not have start state. Cannot sample.");
    }
    auto action_end_map = start_action_map->second.find(action);
    if (action_end_map == start_action_map->second.end()) {
      throw std::runtime_error(
          "MarkovActionChain does not have action from start state. Cannot sample.");
    }
    for (auto& end_value : action_end_map->second) {
      p -= end_value.second;
      if (p <= 0) {
        return end_value.first;
      }
    }
    throw std::runtime_error(
        "End state for specified <start, action> is badly specified. Cannot sample.");
  }

  std::vector<X> domain() const override {
    std::vector<X> result;
    result.reserve(x_.size());
    for (auto& x_x : x_) {
      result.push_back(x_x.first);
    }
    return result;
  }

  std::vector<std::pair<X, U>> range() const override {
    std::vector<std::pair<X, U>> result;
    result.reserve(xu_.size());
    for (auto& xu_xu : xu_) {
      result.push_back(xu_xu.first);
    }
    return result;
  }

  std::shared_ptr<math::probability::discrete::DistributionValueMap<X>> Marginalize(
      const math::probability::RandomDistribution<std::pair<X, U>>& start_action) {
    auto result = std::make_shared<
        math::probability::discrete::DistributionValueMap<X>>();
    for (auto& level0 : p_) {
      X start = level0.first;
      for (auto& level1 : level0.second) {
        U action = level1.first;
        for (auto& level2 : level1.second) {
          X end = level2.first;
          double p0 = level2.second;
          std::pair<X, U> xu(start, action);
          double p1 = start_action.ProbabilityOf(xu);
          double current_end_p = result->ProbabilityOf(end);
          result->Set(end, p0 * p1 + current_end_p);
        }
      }
    }
    result->Normalize();
    return result;
  }

private:
  std::map<X, std::map<U, std::map<X, double>>>p_;
  std::map<X, X> x_;
  std::map<U, U> u_;
  std::map<std::pair<X, U>, std::pair<X, U>> xu_;
};

}
  // namespace robotics
} // namespace cognitoware

#endif /* ROBOTICS_MARKOVACTIONCHAIN_H_ */
