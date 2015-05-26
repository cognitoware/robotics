/*
 * MarkovChain_test.cpp
 *
 *  Copyright (c) 2015, Norman Alan Oursland
 *  All rights reserved.
 */

#include "cognitoware/math/probability/discrete/MarkovChain.h"
#include "gtest/gtest.h"

namespace cognitoware {
namespace math {
namespace probability {
namespace discrete {

enum X {
  x0, x1
};
typedef MarkovChain<X> MC;

TEST(MarkovChainTest, SetGet) {
  MC mc;
  mc.Set(x0, x0, 0.9);
  mc.Set(x1, x0, 0.1);
  mc.Set(x0, x1, 0.2);
  mc.Set(x1, x1, 0.8);
  EXPECT_EQ(0.9, mc.ConditionalProbabilityOf(x0, x0));
  EXPECT_EQ(0.1, mc.ConditionalProbabilityOf(x1, x0));
  EXPECT_EQ(0.2, mc.ConditionalProbabilityOf(x0, x1));
  EXPECT_EQ(0.8, mc.ConditionalProbabilityOf(x1, x1));
}

TEST(MarkovChainTest, LikelihoodOf) {
  MC mc;
  mc.Set(x0, x0, 0.9);
  mc.Set(x1, x0, 0.1);
  mc.Set(x0, x1, 0.2);
  mc.Set(x1, x1, 0.8);

  auto lh0 = mc.LikelihoodOf(x0);
  EXPECT_DOUBLE_EQ(0.9, lh0->ProbabilityOf(x0));
  EXPECT_DOUBLE_EQ(0.2, lh0->ProbabilityOf(x1));
  lh0->Normalize();
  EXPECT_DOUBLE_EQ(9.0 / 11.0, lh0->ProbabilityOf(x0));
  EXPECT_DOUBLE_EQ(2.0 / 11.0, lh0->ProbabilityOf(x1));

  auto lh1 = mc.LikelihoodOf(x1);
  EXPECT_DOUBLE_EQ(0.1, lh1->ProbabilityOf(x0));
  EXPECT_DOUBLE_EQ(0.8, lh1->ProbabilityOf(x1));
  lh1->Normalize();
  EXPECT_DOUBLE_EQ(1.0 / 9.0, lh1->ProbabilityOf(x0));
  EXPECT_DOUBLE_EQ(8.0 / 9.0, lh1->ProbabilityOf(x1));
}

TEST(MarkovChainTest, Marginalize) {
  MC mc;
  mc.Set(x0, x0, 0.9);
  mc.Set(x1, x0, 0.1);
  mc.Set(x0, x1, 0.2);
  mc.Set(x1, x1, 0.8);

  DistributionValueMap<X> belief0(x0, 0.1, x1, 0.9);
  auto belief1 = mc.Marginalize(belief0);
  EXPECT_DOUBLE_EQ(0.27, belief1->ProbabilityOf(x0));
  EXPECT_DOUBLE_EQ(0.73, belief1->ProbabilityOf(x1));
}

TEST(MarkovChainTest, BayesianInference) {
  MC mc;
  mc.Set(x0, x0, 0.9);
  mc.Set(x1, x0, 0.1);
  mc.Set(x0, x1, 0.2);
  mc.Set(x1, x1, 0.8);
  DistributionValueMap<X> belief0(x0, 0.1, x1, 0.9);
  auto belief1 = mc.BayesianInference(x0, belief0);
  EXPECT_DOUBLE_EQ(9.0 / 27.0, belief1->ProbabilityOf(x0));
  EXPECT_DOUBLE_EQ(18.0 / 27.0, belief1->ProbabilityOf(x1));
}

TEST(MarkovChainTest, Reverse) {
  MC mc;
  mc.Set(x0, x0, 1.0);
  mc.Set(x1, x0, 0.0);
  mc.Set(x0, x1, 0.5);
  mc.Set(x1, x1, 0.5);
  EXPECT_EQ(1.0, mc.ConditionalProbabilityOf(x0, x0));
  EXPECT_EQ(0.0, mc.ConditionalProbabilityOf(x1, x0));
  EXPECT_EQ(0.5, mc.ConditionalProbabilityOf(x0, x1));
  EXPECT_EQ(0.5, mc.ConditionalProbabilityOf(x1, x1));

  auto rmc = mc.Reverse();
  EXPECT_DOUBLE_EQ(2.0 / 3.0, rmc->ConditionalProbabilityOf(x0, x0));
  EXPECT_DOUBLE_EQ(1.0 / 3.0, rmc->ConditionalProbabilityOf(x1, x0));
  EXPECT_DOUBLE_EQ(0.0, rmc->ConditionalProbabilityOf(x0, x1));
  EXPECT_DOUBLE_EQ(1.0, rmc->ConditionalProbabilityOf(x1, x1));
}

TEST(MarkovChainTest, domain_range) {
  MC mc;
  mc.Set(x0, x0, 1.0);
  mc.Set(x1, x0, 1.0);
  EXPECT_EQ(std::vector<X>({x0, x1}), mc.domain());
  EXPECT_EQ(std::vector<X>({x0}), mc.range());
}

}  // namespace discrete
}  // namespace probability
}  // namespace math
}  // namespace cognitoware
