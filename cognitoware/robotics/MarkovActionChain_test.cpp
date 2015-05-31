/*
 * MarkovActionChain_test.cpp
 *
 *  Copyright (c) 2015, Norman Alan Oursland
 *  All rights reserved.
 */

#include "cognitoware/math/probability/discrete/DistributionValueMap.h"
#include "cognitoware/robotics/MarkovActionChain.h"
#include "gtest/gtest.h"

#include <vector>

using ::cognitoware::math::probability::discrete::DistributionValueMap;

namespace cognitoware {
namespace robotics {

enum X { x0, x1 };
enum U { u0, u1 };
typedef std::pair<X, U> XU;

TEST(MarkovActionChainTest, GetSet) {
  MarkovActionChain<X, U> mac;
  mac.Set(x0, u0, x0, 0.9);
  mac.Set(x1, u0, x0, 0.1);
  mac.Set(x0, u1, x0, 0.2);
  mac.Set(x1, u1, x0, 0.8);
  mac.Set(x0, u0, x1, 0.3);
  mac.Set(x1, u0, x1, 0.7);
  mac.Set(x0, u1, x1, 0.6);
  mac.Set(x1, u1, x1, 0.4);
  EXPECT_EQ(0.9, mac.ConditionalProbabilityOf(x0, ::std::pair<X, U>(x0, u0)));
  EXPECT_EQ(0.1, mac.ConditionalProbabilityOf(x1, ::std::pair<X, U>(x0, u0)));
  EXPECT_EQ(0.2, mac.ConditionalProbabilityOf(x0, ::std::pair<X, U>(x0, u1)));
  EXPECT_EQ(0.8, mac.ConditionalProbabilityOf(x1, ::std::pair<X, U>(x0, u1)));
  EXPECT_EQ(0.3, mac.ConditionalProbabilityOf(x0, ::std::pair<X, U>(x1, u0)));
  EXPECT_EQ(0.7, mac.ConditionalProbabilityOf(x1, ::std::pair<X, U>(x1, u0)));
  EXPECT_EQ(0.6, mac.ConditionalProbabilityOf(x0, ::std::pair<X, U>(x1, u1)));
  EXPECT_EQ(0.4, mac.ConditionalProbabilityOf(x1, ::std::pair<X, U>(x1, u1)));
}

TEST(MarkovActionChainTest, Sample) {
  MarkovActionChain<X, U> mac;
  mac.Set(x0, u0, x0, 0.9);
  mac.Set(x1, u0, x0, 0.1);

  ASSERT_THROW(mac.SampleCondition(XU(x1, u0), 0.0), std::runtime_error);
  ASSERT_THROW(mac.SampleCondition(XU(x0, u1), 0.0), std::runtime_error);
  EXPECT_EQ(1.0, mac.SumCondition(XU(x0, u0)));
  EXPECT_EQ(x0, mac.SampleCondition(XU(x0, u0), 0.05));
  EXPECT_EQ(x0, mac.SampleCondition(XU(x0, u0), 0.20));
  EXPECT_EQ(x1, mac.SampleCondition(XU(x0, u0), 0.95));
}

TEST(MarkovActionChainTest, domain_range) {
  MarkovActionChain<X, U> mac;
  mac.Set(x0, u0, x0, 0.9);
  mac.Set(x1, u0, x0, 0.1);
  EXPECT_EQ(std::vector<X>({x0, x1}), mac.domain());
  EXPECT_EQ(std::vector<XU>({XU(x0,u0)}), mac.range());
}

TEST(MarkovActionChainTest, Marginalize) {
  MarkovActionChain<X, U> mac;
  mac.Set(x0, u0, x0, 1.0);
  mac.Set(x1, u0, x0, 0.0);
  mac.Set(x0, u1, x0, 0.25);
  mac.Set(x1, u1, x0, 0.75);
  mac.Set(x0, u0, x1, 0.0);
  mac.Set(x1, u0, x1, 1.0);
  mac.Set(x0, u1, x1, 0.75);
  mac.Set(x1, u1, x1, 0.25);

  DistributionValueMap<XU> action0(XU(x0, u0), 0.5, XU(x1, u0), 0.5);
  auto x = mac.Marginalize(action0);
  EXPECT_DOUBLE_EQ(0.5, x->ProbabilityOf(x0));
  EXPECT_DOUBLE_EQ(0.5, x->ProbabilityOf(x1));

  DistributionValueMap<XU> action1(XU(x0, u1), 0.5, XU(x1, u1), 0.5);
  x = mac.Marginalize(action1);
  EXPECT_DOUBLE_EQ(0.5, x->ProbabilityOf(x0));
  EXPECT_DOUBLE_EQ(0.5, x->ProbabilityOf(x1));

  DistributionValueMap<XU> action2(XU(x0, u0), 0.5, XU(x1, u1), 0.5);
  x = mac.Marginalize(action2);
  EXPECT_DOUBLE_EQ(7.0/8.0, x->ProbabilityOf(x0));
  EXPECT_DOUBLE_EQ(1.0/8.0, x->ProbabilityOf(x1));
}

}  // namespace robotics
}  // namespace cognitoware
