/*
 * RangedUniform_test.cpp
 *
 *  Copyright (c) 2015, Norman Alan Oursland
 *  All rights reserved.
 */

#include "cognitoware/math/data/Vector.h"
#include "cognitoware/math/probability/RangedUniform.h"
#include "gtest/gtest.h"

#include <memory>

namespace cognitoware {
namespace math {
namespace probability {

DEFINE_VECTOR1(X);

TEST(RangedUniformTest, ProbabilityOf) {
  auto x = std::make_shared<RangedUniform<X>>(X({0.25}), X({0.75}));
  EXPECT_EQ(0.0, x->ProbabilityOf(0.1));
  EXPECT_EQ(2.0, x->ProbabilityOf(0.3));
  EXPECT_EQ(2.0, x->ProbabilityOf(0.5));
  EXPECT_EQ(2.0, x->ProbabilityOf(0.7));
  EXPECT_EQ(0.0, x->ProbabilityOf(0.9));
}

TEST(RangedUniformTest, Sample) {
  std::default_random_engine generator(0);
  auto x = std::make_shared<RangedUniform<X>>(X({0.25}), X({0.75}));
  for (int i=0; i< 1000; i++) {
    X sample = x->Sample(&generator);
    EXPECT_TRUE(sample[0] < 0.75 && sample[0] > 0.25);
  }
}

}  // namespace probability
}  // namespace math
}  // namespace cognitoware
