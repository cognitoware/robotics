/*
 * Matrix.h
 *
 *  Copyright (c) 2015, Norman Alan Oursland
 *  All rights reserved.
 */

#ifndef COGNITOWARE_MATH_DATA_MATRIX_H_
#define COGNITOWARE_MATH_DATA_MATRIX_H_

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace cognitoware {
namespace math {
namespace data {

class Vector;

class Matrix {
public:
  static Matrix Identity(size_t order) {
    std::vector<double> a(order * order);
    for (std::size_t i = 0; i < a.size(); i += order + 1) {
      a[i] = 1.0;
    }
    return Matrix(order, order, a);
  }
  Matrix();
  Matrix(std::size_t rows, std::size_t cols);
  Matrix(std::size_t rows, std::size_t cols, std::vector<double> values);
  Matrix(std::size_t rows, std::size_t cols, std::vector<double> values,
         bool row_based);
  Matrix(const Matrix& that);
  Matrix(Matrix&& that);

  virtual ~Matrix();

  std::size_t rows() const;
  std::size_t cols() const;
  std::size_t order() const;
  double at(std::size_t row, std::size_t col) const;
  Vector GetColumn(std::size_t col) const;
  Matrix& operator=(Matrix&& that);
  Matrix& operator+=(const Matrix& that);
  Matrix operator+(const Matrix& that) const;
  Matrix operator-(const Matrix& that) const;
  Matrix operator*(const Matrix& that) const;
  Matrix Transpose() const;
  Matrix Inverse() const;
  Matrix Sqrt() const;
  double Determinant() const;
  void ScaleRow(std::size_t r, double k);
  void CombineRow(std::size_t srcRow, std::size_t dstRow, double k);
  Matrix LUDecompositionByGE(int* p_swap_count,
                             std::vector<std::size_t>* p_pivot) const;
  Matrix RoundSymmetry();

private:
  std::size_t GetIndex(std::size_t row, std::size_t col) const;
  static void InitPivots(std::size_t n, std::vector<std::size_t>* p_pivot);
  double PartialPivot(std::vector<std::size_t>* p_pivot, std::size_t k,
                      int* p_swap_count) const;
  bool IsSymmetric() const;
  Matrix CholeskyDecomposition() const;
  std::size_t rows_;
  std::size_t cols_;
  std::vector<double> m_;
  bool row_based_ = true;

  friend Matrix operator*(double scalar, const Matrix& m);
};

std::ostream& operator<<(std::ostream& os, const Matrix& v);

}  // namespace data
}  // namespace math
}  // namespace cognitoware

#endif /* COGNITOWARE_MATH_DATA_MATRIX_H_ */
