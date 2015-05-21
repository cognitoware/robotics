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
  RandomDistribution() {}
  virtual ~RandomDistribution() {}

  virtual double ProbabilityOf(X x) const = 0;
  virtual X Sample(std::default_random_engine* generator) const = 0;

  virtual std::ostream& Write(std::ostream& os) const {
    os << *this;
    return os;
  }

  // TODO: move into utility function
//  virtual double GetEntropy(const vector<X>& samples) const{
//    double result = 0.0;
//    for(auto& x : samples) {
//      double p = ProbabilityOf(x);
//      if( p != 0 ) {
//        result -= p * log2(p);
//      }
//    }
//    return result;
//  }

// TODO: resdesign AliasAs
//  tempate<typename A>
//  virtual A& AsType<A>() = 0;
};

template<typename X>
std::ostream& operator<<(std::ostream& os, const RandomDistribution<X>& p) {
  return p.Write(os);
}

}  // namespace probability
}  // namespace math
}  // namespace cognitoware

#endif /* MATH_PROBABILITY_RANDOMDISTRIBUTION_H_ */
