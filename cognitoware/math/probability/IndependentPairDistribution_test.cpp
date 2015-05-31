/*
 * IndependentPairDistribution_test.cpp
 *
 *  Copyright (c) 2015, Norman Alan Oursland
 *  All rights reserved.
 */

#include "cognitoware/math/probability/discrete/DistributionValueMap.h"
#include "cognitoware/math/probability/IndependentPairDistribution.h"
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
  std::default_random_engine generator(0);
  auto x = std::make_shared<DistributionValueMap<X>>(x0, 0.1, x1, 0.9);
  auto y = std::make_shared<DistributionValueMap<Y>>(y0, 0.2, y1, 0.8);
  auto xy = std::make_shared<IndependentPairDistribution<X, Y>>(x, y);
  double x0y0_count = 0, x1y0_count = 0, x0y1_count = 0, x1y1_count = 0;
  int n = 1000;
  for (int i = 0; i < n; ++i) {
    auto sample = xy->Sample(&generator);
    if (sample == XY(x0, y0)) x0y0_count++;
    if (sample == XY(x0, y1)) x0y1_count++;
    if (sample == XY(x1, y0)) x1y0_count++;
    if (sample == XY(x1, y1)) x1y1_count++;
  }
  std::cout << "Selection counts: "
      << x0y0_count << " "
      << x0y1_count << " "
      << x1y0_count << " "
      << x1y1_count << std::endl;
  EXPECT_TRUE(x0y0_count > 10 && x0y0_count < 30);
  EXPECT_TRUE(x0y1_count > 40 && x0y0_count < 120);
  EXPECT_TRUE(x1y0_count > 90 && x0y0_count < 270);
  EXPECT_TRUE(x1y1_count > 620 && x0y0_count < 820);
}


}  // namespace probability
}  // namespace math
}  // namespace cognitoware
