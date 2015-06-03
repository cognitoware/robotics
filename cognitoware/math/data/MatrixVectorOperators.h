/*
 * MatrixVectorOperators.h
 *
 *  Copyright (c) 2015, Norman Alan Oursland
 *  All rights reserved.
 */

#ifndef MATH_DATA_MATRIXVECTOROPERATORS_H_
#define MATH_DATA_MATRIXVECTOROPERATORS_H_

#include "cognitoware/math/data/Matrix.h"
#include "cognitoware/math/data/Vector.h"

namespace cognitoware {
namespace math {
namespace data {

Vector operator*(const Matrix& m, const Vector& v);

}  // namespace data
}  // namespace math
}  // namespace cognitoware




#endif /* MATH_DATA_MATRIXVECTOROPERATORS_H_ */
