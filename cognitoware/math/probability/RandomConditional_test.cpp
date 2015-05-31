/*
 * RandomConditional_test.cpp
 *
 *  Copyright (c) 2015, Norman Alan Oursland
 *  All rights reserved.
 */

#include "cognitoware/math/data/Vector.h"
#include "cognitoware/math/probability/RandomConditional.h"
#include "cognitoware/math/probability/RandomDistribution.h"
#include "gtest/gtest.h"

using cognitoware::math::data::Vector;

namespace cognitoware {
namespace math {
namespace probability {

DEFINE_VECTOR1(X);
DEFINE_VECTOR1(Y);

class RC : RandomConditional<X, Y> {
  double ConditionalProbabilityOf(const X&, const Y&) const override {
    return 0.0;
  }
};

TEST(RandomConditionalTest, ctor1) {

}

}  // namespace probability
}  // namespace math
}  // namespace cognitoware
