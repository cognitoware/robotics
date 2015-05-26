/*
 * DiscreteDistribution_test.cpp
 *
 *  Copyright (c) 2015, Norman Alan Oursland
 *  All rights reserved.
 */

#include "cognitoware/math/probability/discrete/DiscreteDistribution.h"
#include "gtest/gtest.h"

#include <map>

namespace cognitoware {
namespace math {
namespace probability {
namespace discrete {

enum X {
  x0, x1
};

class TestDD : public DiscreteDistribution<X> {
public:
  template<typename NumberType>
  static X TestSelect(const std::map<X, NumberType>& map, double select) {
    return SelectFromMap(map, select);
  }
};

TEST(DiscreteDistributionTest, SelectFromMap_double) {
  std::map<X, double> map;

  // Cannot select from an empty map
  ASSERT_THROW(TestDD::TestSelect(map, 0.0), std::runtime_error);

  map[x0] = 0.2;
  EXPECT_EQ(x0, TestDD::TestSelect(map, -1.0));
  EXPECT_EQ(x0, TestDD::TestSelect(map, 0.1));
  EXPECT_EQ(x0, TestDD::TestSelect(map, 0.3));

  map[x1] = 0.8;
  EXPECT_EQ(x0, TestDD::TestSelect(map, -1.0));
  EXPECT_EQ(x0, TestDD::TestSelect(map, 0.1));
  EXPECT_EQ(x1, TestDD::TestSelect(map, 0.3));
  EXPECT_EQ(x1, TestDD::TestSelect(map, 2.0));
}

TEST(DiscreteDistributionTest, SelectFromMap_int) {
  std::map<X, int> map;

  // Cannot select from an empty map
  ASSERT_THROW(TestDD::TestSelect(map, 0.0), std::runtime_error);

  map[x0] = 2;
  EXPECT_EQ(x0, TestDD::TestSelect(map, -10.0));
  EXPECT_EQ(x0, TestDD::TestSelect(map, 1.0));
  EXPECT_EQ(x0, TestDD::TestSelect(map, 3.0));

  map[x1] = 8;
  EXPECT_EQ(x0, TestDD::TestSelect(map, -10.0));
  EXPECT_EQ(x0, TestDD::TestSelect(map, 1.0));
  EXPECT_EQ(x1, TestDD::TestSelect(map, 3.0));
  EXPECT_EQ(x1, TestDD::TestSelect(map, 20.0));
}

}  // namespace discrete
}  // namespace probability
}  // namespace math
}  // namespace cognitoware
