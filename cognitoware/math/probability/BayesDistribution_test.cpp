/*
 * BayesDistribution_test.cpp
 *
 *  Copyright (c) 2015, Norman Alan Oursland
 *  All rights reserved.
 */

#include "cognitoware/math/probability/BayesDistribution.h"
#include "cognitoware/math/data/Vector.h"
#include "gtest/gtest.h"

using cognitoware::math::data::Vector;

namespace cognitoware {
namespace math {
namespace probability {

DEFINE_VECTOR1(X);
DEFINE_VECTOR1(Y);

class BD1 : public BayesDistribution<X, Y> {

};

TEST(BayesDistributionTest, ctor1) {
//  X belief;
//
//  BD1 bd(belief, sensor_model);
}

}  // namespace probability
}  // namespace math
}  // namespace cognitoware
