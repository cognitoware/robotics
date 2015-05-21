/*
 * RandomDistribution_test.cpp
 *
 *  Created on: Jan 18, 2015
 *      Author: Alan Oursland
 */

#include "cognitoware/math/probability/RandomDistribution.h"
#include "cognitoware/math/data/Vector.h"
#include "gtest/gtest.h"

using cognitoware::math::data::Vector;

namespace cognitoware {
namespace math {
namespace probability {

DEFINE_VECTOR1(X);

class Impulse : RandomDistribution<X> {
public:
  double ProbabilityOf(X x) const override {
    if (x[0] == 0.0) return 1.0;
    return 0.0;
  }
  X Sample(std::default_random_engine* ) const override {
    return 0.0;
  }
};

TEST(RandomDistributionTest, ctor1) {
  std::default_random_engine generator(0);
  Impulse d;
  ASSERT_EQ(1.0, d.ProbabilityOf(0.0));
  ASSERT_EQ(0.0, d.ProbabilityOf(0.5));
  ASSERT_EQ(X(0.0), d.Sample(&generator));
}

}  // namespace probability
}  // namespace math
}  // namespace cognitoware
