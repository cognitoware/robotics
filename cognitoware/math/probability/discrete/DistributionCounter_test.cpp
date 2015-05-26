/*
 * DistributionCounter_test.cpp
 *
 *  Copyright (c) 2015, Norman Alan Oursland
 *  All rights reserved.
 */

#include "cognitoware/math/probability/discrete/DistributionCounter.h"
#include "gtest/gtest.h"

#include <iostream>

namespace cognitoware {
namespace math {
namespace probability {
namespace discrete {

enum X {
  x0, x1
};

typedef DistributionCounter<X> DC;

TEST(DistributionCounterTest, ProbabilityOf) {
  DC dc;
  EXPECT_EQ(0, dc.ProbabilityOf(x0));
  EXPECT_EQ(0, dc.ProbabilityOf(x1));

  dc.AddObservation(x0);
  EXPECT_EQ(1, dc.ProbabilityOf(x0));
  EXPECT_EQ(0, dc.ProbabilityOf(x1));

  dc.AddObservation(x0);
  dc.AddObservation(x0);
  EXPECT_EQ(1, dc.ProbabilityOf(x0));
  EXPECT_EQ(0, dc.ProbabilityOf(x1));

  dc.AddObservation(x1);
  EXPECT_EQ(0.75, dc.ProbabilityOf(x0));
  EXPECT_EQ(0.25, dc.ProbabilityOf(x1));
}

TEST(DistributionCounterTest, Sample) {
  std::default_random_engine generator(0);
  DC dc;
  dc.AddObservation(x0);
  dc.AddObservation(x0);
  dc.AddObservation(x0);
  dc.AddObservation(x1);
  double x0_count = 0, x1_count = 0;
  for (int i = 0; i < 1000; ++i) {
    X sample = dc.Sample(&generator);
    if (sample == x0) x0_count++;
    if (sample == x1) x1_count++;
  }
  double r = x0_count / x1_count;
  std::cout << "Selection ratio is " << r << " : 1" << std::endl;
  EXPECT_TRUE(r > 2.5);
  EXPECT_TRUE(r < 3.5);
}
}  // namespace discrete
}  // namespace probability
}  // namespace math
}  // namespace cognitoware
