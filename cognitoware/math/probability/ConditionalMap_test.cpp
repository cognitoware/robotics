/*
 * ConditionalMap_test.cpp
 *
 *  Copyright (c) 2015, Norman Alan Oursland
 *  All rights reserved.
 */

#include "cognitoware/math/data/Vector.h"
#include "cognitoware/math/probability/ConditionalMap.h"
#include "cognitoware/math/probability/RangedUniform.h"
#include "gtest/gtest.h"

using cognitoware::math::data::Vector;

namespace cognitoware {
namespace math {
namespace probability {

DEFINE_VECTOR1(X);
enum Y {
  y0, y1
};

TEST(ConditionalMapTest, GetSet) {
  ConditionalMap<X, Y> px_given_y;
  px_given_y.Set(y0, std::make_shared<RangedUniform<X>>(X(0.5), X(1.0)));
  px_given_y.Set(y1, std::make_shared<RangedUniform<X>>(X(0.0), X(2.0)));
  EXPECT_EQ(0.0, px_given_y.ConditionalProbabilityOf(0.4, y0));
  EXPECT_EQ(2.0, px_given_y.ConditionalProbabilityOf(0.8, y0));
  EXPECT_EQ(0.0, px_given_y.ConditionalProbabilityOf(1.5, y0));
  EXPECT_EQ(0.0, px_given_y.ConditionalProbabilityOf(-1.0, y1));
  EXPECT_EQ(0.5, px_given_y.ConditionalProbabilityOf(1.5, y1));
  EXPECT_EQ(0.0, px_given_y.ConditionalProbabilityOf(3.0, y1));
}

}  // namespace probability
}  // namespace math
}  // namespace cognitoware
