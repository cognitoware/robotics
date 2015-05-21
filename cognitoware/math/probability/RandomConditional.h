/*
 * RandomConditional.h
 *
 *  Created on: Jan 17, 2015
 *      Author: Alan Oursland
 */

#ifndef COGNITOWARE_COGNITOWARE_MATH_PROBABILITY_RANDOMCONDITIONAL_H_
#define COGNITOWARE_COGNITOWARE_MATH_PROBABILITY_RANDOMCONDITIONAL_H_

#include "cognitoware/math/probability/RandomDistribution.h"

#include <memory>

namespace cognitoware {
namespace math {
namespace probability {

template<typename X, typename Y>
class RandomConditional : public std::enable_shared_from_this<RandomConditional<X, Y>> {
public:
  RandomConditional() {}
  virtual ~RandomConditional() {}

  // TODO: Uncomment definitions.
  virtual double ConditionalProbabilityOf(X x, Y y) const = 0;
//  virtual shared_ptr<RandomDistribution<Y>> LikelihoodOf(X x) const = 0;
//  virtual shared_ptr<RandomDistribution<X>> ConditionBy(Y y) const = 0;
  virtual std::shared_ptr<RandomDistribution<Y>> BayesianInference(
      X observation, std::shared_ptr<const RandomDistribution<Y>> belief) const = 0;
//  virtual shared_ptr<RandomDistribution<X>> Marginalize(const RandomDistribution<Y>& y) const = 0;
//  virtual double Marginalize(X x, const RandomDistribution<Y>& y, Y z) = 0;

//  virtual shared_ptr<RandomDistribution<Y>> BayesianInference(
//      X data, const RandomDistribution<Y>& prior) const = 0;
// {
//    return make_shared<BayesDistribution<Y, X>>(prior, this, data);
//  }
};

}  // namespace probability
}  // namespace math
}  // namespace cognitoware

#endif /* COGNITOWARE_COGNITOWARE_MATH_PROBABILITY_RANDOMCONDITIONAL_H_ */
