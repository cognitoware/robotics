/*
 * ConditionalValueMap_test.cpp
 *
 *  Copyright (c) 2015, Norman Alan Oursland
 *  All rights reserved.
 */

#include "cognitoware/math/probability/discrete/ConditionalValueMap.h"
#include "gtest/gtest.h"

namespace cognitoware {
namespace math {
namespace probability {
namespace discrete {

enum X {
  x0, x1
};
enum Y {
  y0, y1
};
enum Z {
  z0, z1
};

typedef ConditionalValueMap<X, Y> CvmXY;
typedef ConditionalValueMap<Y, Z> CvmYZ;

TEST(ConditionalValueMapTest, DomainRange) {
  CvmXY xy;
  xy.Set(x0, y0, 1.0);
  xy.Set(x1, y0, 1.0);
  EXPECT_EQ(std::vector<X>({x0, x1}), xy.domain());
  EXPECT_EQ(std::vector<Y>({y0}), xy.range());
}

TEST(ConditionalValueMapTest, SetGet) {
  CvmXY xy;
  xy.Set(x0, y0, 0.9);
  xy.Set(x1, y0, 0.1);
  xy.Set(x0, y1, 0.2);
  xy.Set(x1, y1, 0.8);
  EXPECT_EQ(0.9, xy.ConditionalProbabilityOf(x0, y0));
  EXPECT_EQ(0.1, xy.ConditionalProbabilityOf(x1, y0));
  EXPECT_EQ(0.2, xy.ConditionalProbabilityOf(x0, y1));
  EXPECT_EQ(0.8, xy.ConditionalProbabilityOf(x1, y1));
}

TEST(ConditionalValueMapTest, LikelihoodOf) {
  CvmXY xy;
  xy.Set(x0, y0, 0.9);
  xy.Set(x1, y0, 0.1);
  xy.Set(x0, y1, 0.2);
  xy.Set(x1, y1, 0.8);

  auto lh0 = xy.LikelihoodOf(x0);
  EXPECT_DOUBLE_EQ(0.9, lh0->ProbabilityOf(y0));
  EXPECT_DOUBLE_EQ(0.2, lh0->ProbabilityOf(y1));
  lh0->Normalize();
  EXPECT_DOUBLE_EQ(9.0 / 11.0, lh0->ProbabilityOf(y0));
  EXPECT_DOUBLE_EQ(2.0 / 11.0, lh0->ProbabilityOf(y1));

  auto lh1 = xy.LikelihoodOf(x1);
  EXPECT_DOUBLE_EQ(0.1, lh1->ProbabilityOf(y0));
  EXPECT_DOUBLE_EQ(0.8, lh1->ProbabilityOf(y1));
  lh1->Normalize();
  EXPECT_DOUBLE_EQ(1.0 / 9.0, lh1->ProbabilityOf(y0));
  EXPECT_DOUBLE_EQ(8.0 / 9.0, lh1->ProbabilityOf(y1));
}

TEST(ConditionalValueMapTest, BayesianInference) {
  CvmXY xy;
  xy.Set(x0, y0, 0.9);
  xy.Set(x1, y0, 0.1);
  xy.Set(x0, y1, 0.2);
  xy.Set(x1, y1, 0.8);
  DistributionValueMap<Y> belief0(y0, 0.1, y1, 0.9);
  auto belief1 = xy.BayesianInference(x0, belief0);
  EXPECT_DOUBLE_EQ(9.0 / 27.0, belief1->ProbabilityOf(y0));
  EXPECT_DOUBLE_EQ(18.0 / 27.0, belief1->ProbabilityOf(y1));
}

TEST(ConditionalValueMapTest, Marginalize) {
  CvmXY xy;
  xy.Set(x0, y0, 0.9);
  xy.Set(x1, y0, 0.1);
  xy.Set(x0, y1, 0.2);
  xy.Set(x1, y1, 0.8);

  CvmYZ yz;
  yz.Set(y0, z0, 0.9);
  yz.Set(y1, z0, 0.1);
  yz.Set(y0, z1, 0.2);
  yz.Set(y1, z1, 0.8);

  auto xz = xy.Marginalize(yz);
  EXPECT_DOUBLE_EQ(0.83, xz->ConditionalProbabilityOf(x0, z0));
  EXPECT_DOUBLE_EQ(0.17, xz->ConditionalProbabilityOf(x1, z0));
  EXPECT_DOUBLE_EQ(0.34, xz->ConditionalProbabilityOf(x0, z1));
  EXPECT_DOUBLE_EQ(0.66, xz->ConditionalProbabilityOf(x1, z1));
}

TEST(ConditionalValueMapTest, SampleLikelihood) {
  CvmXY xy;
  xy.Set(x0, y0, 0.9);
  xy.Set(x1, y0, 0.1);
  xy.Set(x0, y1, 0.2);
  xy.Set(x1, y1, 0.8);

  EXPECT_EQ(1.1, xy.SumLikelihood(x0));
  EXPECT_EQ(0.9, xy.SumLikelihood(x1));

  EXPECT_EQ(y0, xy.SampleLikelihood(x0, 0.85));
  EXPECT_EQ(y1, xy.SampleLikelihood(x0, 0.95));
  EXPECT_EQ(y0, xy.SampleLikelihood(x1, 0.05));
  EXPECT_EQ(y1, xy.SampleLikelihood(x1, 0.15));
}

}  // namespace discrete
}  // namespace probability
}  // namespace math
}  // namespace cognitoware
