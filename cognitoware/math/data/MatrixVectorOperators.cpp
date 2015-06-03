/*
 * MatrixVectorOperators.cpp
 *
 *  Copyright (c) 2015, Norman Alan Oursland
 *  All rights reserved.
 */

#include "cognitoware/math/data/Matrix.h"
#include "cognitoware/math/data/MatrixVectorOperators.h"
#include "cognitoware/math/data/Vector.h"

#include <stdexcept>
#include <iostream>

namespace cognitoware {
namespace math {
namespace data {

Vector operator*(const Matrix& m, const Vector& v) {
  if (m.cols() != v.order()) {
    throw std::runtime_error("Matrix and Vector sizes not compatible.");
  }
  std::vector<double> result(m.rows());
  for (std::size_t row = 0; row < m.rows(); row++) {
    double value = 0.0;
    for (std::size_t col = 0; col < m.cols(); col++) {
      value += m.at(row, col) * v[col];
    }
    result[row] = value;
  }
  return Vector(result);
}

}  // namespace data
}  // namespace math
}  // namespace cognitoware

