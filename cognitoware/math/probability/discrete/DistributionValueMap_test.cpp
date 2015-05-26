/*
 * DistributionValueMap_test.cpp
 *
 *  Copyright (c) 2015, Norman Alan Oursland
 *  All rights reserved.
 */

#include "cognitoware/math/probability/discrete/DistributionValueMap.h"
#include "gtest/gtest.h"

namespace cognitoware {
namespace math {
namespace probability {
namespace discrete {

enum X {
  x0, x1
};

typedef DistributionValueMap<X> DVM;

TEST(DistributionValueMapTest, ctor1) {
  DVM dvm(x0, 1, x1, 3);
  EXPECT_EQ(0.25, dvm.ProbabilityOf(x0));
  EXPECT_EQ(0.75, dvm.ProbabilityOf(x1));
}

TEST(DistributionValueMapTest, ctor2) {
  DVM dvm({x0, x1});
  EXPECT_EQ(0.5, dvm.ProbabilityOf(x0));
  EXPECT_EQ(0.5, dvm.ProbabilityOf(x1));
}

TEST(DistributionValueMapTest, GetSet) {
  DVM dvm;
  EXPECT_EQ(0.0, dvm.ProbabilityOf(x0));
  EXPECT_EQ(0.0, dvm.ProbabilityOf(x1));

  dvm.Set(x0, 0.2);
  EXPECT_EQ(0.2, dvm.ProbabilityOf(x0));
  EXPECT_EQ(0.0, dvm.ProbabilityOf(x1));

  dvm.Set(x1, 0.8);
  EXPECT_EQ(0.2, dvm.ProbabilityOf(x0));
  EXPECT_EQ(0.8, dvm.ProbabilityOf(x1));
}

TEST(DistributionValueMapTest, Sample) {
  std::default_random_engine generator(0);
  DVM dvm(x0, 0.75, x1, 0.25);
  double x0_count = 0, x1_count = 0;
  for (int i = 0; i < 1000; ++i) {
    X sample = dvm.Sample(&generator);
    if (sample == x0) x0_count++;
    if (sample == x1) x1_count++;
  }
  double r = x0_count / x1_count;
  std::cout << "Selection ratio is " << r << " : 1" << std::endl;
  EXPECT_TRUE(r > 2.5);
  EXPECT_TRUE(r < 3.5);
}

TEST(DistributionValueMapTest, Normalize) {
  DVM dvm;
  dvm.Set(x0, 1);
  dvm.Set(x1, 4);
  EXPECT_EQ(1.0, dvm.ProbabilityOf(x0));
  EXPECT_EQ(4.0, dvm.ProbabilityOf(x1));

  dvm.Normalize();
  EXPECT_EQ(0.2, dvm.ProbabilityOf(x0));
  EXPECT_EQ(0.8, dvm.ProbabilityOf(x1));
}

TEST(DistributionValueMapTest, GetEntropy) {
  DVM dvm0({x0, x1});
  EXPECT_EQ(1.0, dvm0.GetEntropy());

  DVM dvm1({x0});
  EXPECT_EQ(0.0, dvm1.GetEntropy());
}

TEST(DistributionValueMapTest, Product) {
  DVM dvm0(x0, 0.25, x1, 0.75);
  DVM dvm1(x0, 0.75, x1, 0.25);

  auto result1 = dvm0.Product(dvm1);
  EXPECT_EQ(0.5, result1->ProbabilityOf(x0));
  EXPECT_EQ(0.5, result1->ProbabilityOf(x1));

  auto result2 = dvm0.Product(dvm0);
  EXPECT_EQ(0.1, result2->ProbabilityOf(x0));
  EXPECT_EQ(0.9, result2->ProbabilityOf(x1));

}

}  // namespace discrete
}  // namespace probability
}  // namespace math
}  // namespace cognitoware
