/*
 * Vector_test.cpp
 *
 *  Copyright (c) 2015, Norman Alan Oursland
 *  All rights reserved.
 *
 */

#include "cognitoware/math/data/Vector.h"
#include "gtest/gtest.h"

namespace cognitoware {
namespace math {
namespace data {

DEFINE_VECTOR1(VT1);

TEST(VectorTest, ctor1) {
  VT1 v;
  ASSERT_EQ(std::size_t(1), v.order());
  ASSERT_EQ(0.0, v.at(0));
  ASSERT_EQ(0.0, v[0]);

  VT1 v2(1.0);
  ASSERT_EQ(std::size_t(1), v2.order());
  ASSERT_EQ(1.0, v2[0]);
}

TEST(VectorTest, ops1) {
  VT1 v1;
  VT1 v2;
  VT1 v3;
  v1[0] = 2.0;
  v2[0] = 3.0;
  v3[0] = 3.0;
  ASSERT_EQ(VT1(1.0), (v2 - v1));
  ASSERT_EQ(VT1(5.0), (v2 + v1));
  ASSERT_EQ(VT1(2.0), (v2 + v1 - v2));
  ASSERT_EQ(6.0, v2.Dot(v1));
  ASSERT_EQ(6.0, v1.Dot(v2));
  ASSERT_EQ(9.0, v2.Dot(v2));
  ASSERT_TRUE(v1 < v2);
  ASSERT_FALSE(v2 < v1);
  ASSERT_TRUE(v2 > v1);
  ASSERT_FALSE(v1 > v2);
  ASSERT_TRUE(v2 == v2);
  ASSERT_TRUE(v2 == v3);
  ASSERT_FALSE(v2 == v1);
  VT1 v4(v2);
  ASSERT_TRUE(v4 == v2);
  v1.copyAssign(v3);
  ASSERT_TRUE(v3 == v1);
  v1.moveAssign(std::move(v4));
  ASSERT_TRUE(v2 == v1);
  VT1 v5 = v1 + v2;
  ASSERT_EQ(VT1(6.0), v5);
}

}  // namespace data
}  // namespace math
}  // namespace cognitoware
