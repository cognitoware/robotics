/*
 * BayesDistribution_test.cpp
 *
 *  Copyright (c) 2015, Norman Alan Oursland
 *  All rights reserved.
 */

#include "cognitoware/math/data/Vector.h"
#include "cognitoware/math/probability/BayesDistribution.h"
#include "cognitoware/math/probability/discrete/DistributionValueMap.h"
#include "cognitoware/math/probability/RandomConditional.h"
#include "cognitoware/math/probability/RangedUniform.h"
#include "gtest/gtest.h"

#include <memory>

using cognitoware::math::data::Vector;
using cognitoware::math::probability::discrete::DistributionValueMap;

namespace cognitoware {
namespace math {
namespace probability {

DEFINE_VECTOR1(X);
DEFINE_VECTOR1(Z);

// A sensor model that says that X is within 0.25 units observed Z
class SensorModel : public RandomConditional<Z, X> {
public:
  double ConditionalProbabilityOf(const Z& z, const X& x) const override {
    if (fabs(z[0] - x[0]) < 0.25) return 2.0;
    return 0.0;
  }
};

typedef BayesDistribution<X, Z> BayesXZ;

TEST(BayesDistributionTest, ProbabilityOf) {
  auto belief = std::make_shared<RangedUniform<X>>(X({0}), X({2}));
  auto sensor_model = std::make_shared<SensorModel>();
  auto bd = std::make_shared<BayesXZ>(belief, sensor_model, 0.5);
  // Probabilities are not normalized. They should be treated proportionally
  // and not as exact probabilities.
  EXPECT_EQ(0.0, bd->ProbabilityOf(0.1));
  EXPECT_EQ(1.0, bd->ProbabilityOf(0.3));
  EXPECT_EQ(1.0, bd->ProbabilityOf(0.5));
  EXPECT_EQ(1.0, bd->ProbabilityOf(0.7));
  EXPECT_EQ(0.0, bd->ProbabilityOf(0.9));
}

TEST(BayesDistributionTest, Sample) {
  std::default_random_engine generator(0);
  auto belief = std::make_shared<RangedUniform<X>>(X({0}), X({2}));
  auto sensor_model = std::make_shared<SensorModel>();
  auto bd = std::make_shared<BayesXZ>(belief, sensor_model, 0.5);
  // Sample is not implemented in Bayes Distribution
  ASSERT_THROW(bd->Sample(&generator), std::runtime_error);
}

TEST(BayesDistributionTest, ChainObservation) {
  auto belief = std::make_shared<RangedUniform<X>>(X({0}), X({2}));
  auto sensor_model = std::make_shared<SensorModel>();
  auto bd = std::make_shared<BayesXZ>(belief, sensor_model, 0.5);
  bd = bd->ChainObservation(0.9);
  // Probabilities are not normalized. They should be treated proportionally
  // and not as exact probabilities.
  EXPECT_EQ(0.0, bd->ProbabilityOf(0.1));
  EXPECT_EQ(0.0, bd->ProbabilityOf(0.3));
  EXPECT_EQ(0.0, bd->ProbabilityOf(0.5));
  EXPECT_EQ(2.0, bd->ProbabilityOf(0.7));
  EXPECT_EQ(0.0, bd->ProbabilityOf(0.9));
}

TEST(BayesDistributionTest, EstimateNormalizer) {
  auto belief = std::make_shared<RangedUniform<X>>(X({0}), X({2}));
  auto sensor_model = std::make_shared<SensorModel>();
  auto bd = std::make_shared<BayesXZ>(belief, sensor_model, 0.5);
  std::vector<X> samples;
  double dx = 0.02;
  samples.reserve((long)(4/dx));
  for(double x = -1.0; x < 3.0; x += dx) {
    samples.push_back(x);
  }
  EXPECT_EQ(0.5, bd->EstimateNormalizer(samples, 0.02));
  EXPECT_EQ(0.5, bd->normalizer());
  EXPECT_EQ(0.0, bd->ProbabilityOf(0.1));
  EXPECT_EQ(2.0, bd->ProbabilityOf(0.3));
  EXPECT_EQ(2.0, bd->ProbabilityOf(0.5));
  EXPECT_EQ(2.0, bd->ProbabilityOf(0.7));
  EXPECT_EQ(0.0, bd->ProbabilityOf(0.9));
}

TEST(BayesDistributionTest, Properties) {
  auto belief = std::make_shared<RangedUniform<X>>(X({0}), X({2}));
  auto sensor_model = std::make_shared<SensorModel>();
  auto bd = std::make_shared<BayesXZ>(belief, sensor_model, 0.5);
  EXPECT_EQ(sensor_model.get(), &bd->sensor_model());
  EXPECT_EQ(Z(0.5), bd->data());
}

}  // namespace probability
}  // namespace math
}  // namespace cognitoware
