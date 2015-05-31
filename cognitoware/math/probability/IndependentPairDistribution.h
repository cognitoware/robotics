/*
 * IndependentPairDistribution.h
 *
 *  Copyright (c) 2015, Norman Alan Oursland
 *  All rights reserved.
 */

#ifndef COGNITOWARE_MATH_PROBABILITY_INDEPENDENTPAIRDISTRIBUTION_H_
#define COGNITOWARE_MATH_PROBABILITY_INDEPENDENTPAIRDISTRIBUTION_H_

#include "cognitoware/math/probability/RandomDistribution.h"

#include <memory>
#include <utility>

namespace cognitoware {
namespace math {
namespace probability {

template<typename A, typename B>
class IndependentPairDistribution : public RandomDistribution<std::pair<A, B>> {
public:
  typedef std::pair<A, B> AB;

  IndependentPairDistribution(std::shared_ptr<RandomDistribution<A>> a,
                              std::shared_ptr<RandomDistribution<B>> b) :
      a_(a), b_(b) {

  }

  virtual ~IndependentPairDistribution() {
  }

  double ProbabilityOf(const AB& t) const override {
    return a_->ProbabilityOf(t.first) * b_->ProbabilityOf(t.second);
  }

  AB Sample(std::default_random_engine* generator) const override {
    return AB(a_->Sample(generator), b_->Sample(generator));
  }

private:
  std::shared_ptr<RandomDistribution<A>> a_;
  std::shared_ptr<RandomDistribution<B>> b_;

};

}  // namespace probability
}  // namespace math
}  // namespace cognitoware

#endif /* COGNITOWARE_MATH_PROBABILITY_INDEPENDENTPAIRDISTRIBUTION_H_ */
