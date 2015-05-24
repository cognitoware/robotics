/*
 * RandomDistribution.h
 *
 *  Copyright (c) 2015, Norman Alan Oursland
 *  All rights reserved.
 */

#ifndef MATH_PROBABILITY_RANDOMDISTRIBUTION_H_
#define MATH_PROBABILITY_RANDOMDISTRIBUTION_H_

#include <random>

namespace cognitoware {
namespace math {
namespace probability {

template<typename X>
class RandomDistribution {
public:
  RandomDistribution() {
  }

  virtual ~RandomDistribution() {
  }

  virtual double ProbabilityOf(const X& x) const = 0;

  virtual X Sample(std::default_random_engine* generator) const = 0;
};
}  // namespace probability
}  // namespace math
}  // namespace cognitoware

#endif /* MATH_PROBABILITY_RANDOMDISTRIBUTION_H_ */
