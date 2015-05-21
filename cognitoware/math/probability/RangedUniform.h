/*
 * RangedUniform.h
 *
 *  Created on: Jan 18, 2015
 *      Author: Alan Oursland
 */

#ifndef MATH_PROBABILITY_RANGEDUNIFORM_H_
#define MATH_PROBABILITY_RANGEDUNIFORM_H_

#include "cognitoware/math/probability/RandomDistribution.h"

#include "cognitoware/math/data/Vector.h"
#include "cognitoware/math/data/VectorRange.h"

#include <type_traits>
#include <utility>

using ::cognitoware::math::data::Vector;
using ::cognitoware::math::data::VectorRange;

namespace cognitoware {
namespace math {
namespace probability {

template<typename X>
class RangedUniform : public RandomDistribution<X> {
public:
  RangedUniform(X min, X max) : range_(std::move(min), std::move(max)) {
    static_assert(std::is_base_of<Vector<X, X::Order>, X>::value, "X must derive from Vector");
  }
  ~RangedUniform() override {}
  double ProbabilityOf(X x) const override {
    if( range_.Contains(x) )
      return 1.0 / range_.area();
    return 0.0;
  }
  X Sample(std::default_random_engine* generator) const override {
    return range_.Sample(generator);
  }
private:
  VectorRange<X> range_;
};

}  // namespace probability
}  // namespace math
}  // namespace cognitoware

#endif /* MATH_PROBABILITY_RANGEDUNIFORM_H_ */
