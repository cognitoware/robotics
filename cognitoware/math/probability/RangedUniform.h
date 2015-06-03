/*
 * RangedUniform.h
 *
 *  Copyright (c) 2015, Norman Alan Oursland
 *  All rights reserved.
 */

#ifndef COGNITOWARE_MATH_PROBABILITY_RANGEDUNIFORM_H_
#define COGNITOWARE_MATH_PROBABILITY_RANGEDUNIFORM_H_

#include "cognitoware/math/data/Vector.h"
#include "cognitoware/math/data/VectorRange.h"
#include "cognitoware/math/probability/RandomDistribution.h"

#include <type_traits>
#include <utility>

namespace cognitoware {
namespace math {
namespace probability {

template<typename X>
class RangedUniform : public RandomDistribution<X> {
public:
  RangedUniform(X min, X max) :
      range_(std::move(min), std::move(max)) {
    static_assert(std::is_base_of<data::Vector, X>::value, "X must derive from Vector");
  }

  ~RangedUniform() override {
  }

  double ProbabilityOf(const X& x) const override {
    if (range_.Contains(x)) return 1.0 / range_.area();
    return 0.0;
  }

  X Sample(std::default_random_engine* generator) const override {
    return range_.Sample(generator);
  }

private:
  data::VectorRange<X> range_;
};

}  // namespace probability
}  // namespace math
}  // namespace cognitoware

#endif /* COGNITOWARE_MATH_PROBABILITY_RANGEDUNIFORM_H_ */
