/*
 * IndependentPairDistribution_test.cpp
 *
 *  Copyright (c) 2015, Norman Alan Oursland
 *  All rights reserved.
 */

#include "cognitoware/math/probability/IndependentPairDistribution.h"
#include "cognitoware/math/probability/discrete/DistributionValueMap.h"
#include "gtest/gtest.h"

#include <memory>

using ::cognitoware::math::probability::discrete::DistributionValueMap;

namespace cognitoware {
namespace math {
namespace probability {

enum X { x0, x1 };
enum Y { y0, y1 };
typedef std::pair<X, Y> XY;

TEST(IndependentPairDistributionTest, ProbabilityOf) {
  auto x = std::make_shared<DistributionValueMap<X>>(x0, 0.1, x1, 0.9);
  auto y = std::make_shared<DistributionValueMap<Y>>(y0, 0.2, y1, 0.8);
  auto xy = std::make_shared<IndependentPairDistribution<X, Y>>(x, y);
  EXPECT_DOUBLE_EQ(0.02, xy->ProbabilityOf(XY(x0, y0)));
  EXPECT_DOUBLE_EQ(0.08, xy->ProbabilityOf(XY(x0, y1)));
  EXPECT_DOUBLE_EQ(0.18, xy->ProbabilityOf(XY(x1, y0)));
  EXPECT_DOUBLE_EQ(0.72, xy->ProbabilityOf(XY(x1, y1)));
}

TEST(IndependentPairDistributionTest, Sample) {
  // TODO: Test with Sample refactoring
}


}  // namespace probability
}  // namespace math
}  // namespace cognitoware
