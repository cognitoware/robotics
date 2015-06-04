/*
 * Matrix.h
 *
 *  Copyright (c) 2015, Norman Alan Oursland
 *  All rights reserved.
 */

#ifndef COGNITOWARE_MATH_DATA_MATRIX_H_
#define COGNITOWARE_MATH_DATA_MATRIX_H_

#include <cmath>
#include <vector>
#include <stdexcept>

namespace cognitoware {
namespace math {
namespace data {

class Matrix {
public:
  Matrix() :
      rows_(0), cols_(0), m_(), row_based_(true) {
  }

  Matrix(std::size_t rows, std::size_t cols) :
      rows_(rows), cols_(cols), m_(rows * cols), row_based_(true) {
  }

  Matrix(std::size_t rows, std::size_t cols, std::vector<double> values) :
      rows_(rows), cols_(cols), m_(values), row_based_(true) {
  }

  Matrix(std::size_t rows, std::size_t cols, std::vector<double> values,
         bool row_based) :
      rows_(rows), cols_(cols), m_(values), row_based_(row_based) {
  }

  Matrix(const Matrix& that) :
      rows_(that.rows_),
          cols_(that.cols_),
          m_(that.m_),
          row_based_(that.row_based_) {
  }

  Matrix(Matrix&& that) :
      rows_(that.rows_),
          cols_(that.cols_),
          m_(std::move(that.m_)),
          row_based_(that.row_based_) {
  }

  virtual ~Matrix() {
  }

  std::size_t rows() const {
    return rows_;
  }
  std::size_t cols() const {
    return cols_;
  }
  std::size_t order() const {
    if (rows_ != cols_) {
      throw std::runtime_error("Matrix must be square.");
    }
    return rows_;
  }

  Matrix& operator=(Matrix&& that) {
    rows_ = that.rows_;
    cols_ = that.cols_;
    m_ = std::move(that.m_);
    row_based_ = that.row_based_;
    return *this;
  }

  Matrix operator*(const Matrix& that) const {
    if (cols_ != that.rows_) {
      throw std::runtime_error("Matrix sizes not compatible.");
    }
    Matrix result(rows_, that.cols_);
    for (std::size_t col = 0; col < that.cols_; col++) {
      for (std::size_t row = 0; row < rows_; row++) {
        for (std::size_t k = 0; k < cols_; k++) {
          result.m_[result.GetIndex(row, col)] += at(row, k) * that.at(k, col);
        }
      }
    }
    return result;
  }

  Matrix& operator+=(const Matrix& that) {
    if (rows_ != that.rows_ || cols_ != that.cols_) {
      throw std::runtime_error("Matrix sizes not compatible.");
    }

    if (row_based_ == row_based_) {
      for (std::size_t i = 0; i < m_.size(); i++) {
        m_[i] += that.m_[i];
      }
    } else {
      for (std::size_t row = 0; row < rows(); row++) {
        for (std::size_t col = 0; col < cols(); col++) {
          std::size_t this_index = GetIndex(row, col);
          std::size_t that_index = that.GetIndex(row, col);
          m_[this_index] += that.m_[that_index];
        }
      }
    }

    return *this;
  }

  Matrix Transpose() const {
    return Matrix(cols_, rows_, m_, !row_based_);
  }

  Matrix Inverse() const {
    if (rows_ != cols_) {
      throw std::runtime_error("Matrix must be square.");
    }
    // set up mean as identity
    Matrix result(rows_, cols_);
    for (std::size_t i = 0; i < cols_; i++) {
      result.m_[result.GetIndex(i, i)] = 1.0;
    }

    Matrix working(rows_, cols_, m_);  // creates a copy of m_
    // Eliminate L
    for (std::size_t col = 0; col < cols_; col++) {
      double diag = working.at(col, col);
      for (std::size_t row = col + 1; row < rows_; row++) {
        double target = working.at(row, col);
        double a = -target / diag;
        working.CombineRow(col, row, a);
        result.CombineRow(col, row, a);
      }
      working.ScaleRow(col, 1.0 / diag);
      result.ScaleRow(col, 1.0 / diag);
    }
    // Eliminate U
    for (std::size_t col = cols_ - 1; col >= 1; col--) {
      double diag = working.at(col, col);  // 1.0
      for (std::size_t row_plus_one = col; row_plus_one > 0; row_plus_one--) {
        std::size_t row = row_plus_one - 1;
        double target = working.at(row, col);
        double a = -target / diag;
        working.CombineRow(col, row, a);
        result.CombineRow(col, row, a);
      }
    }
    return result;
  }

  Matrix Sqrt() const {
    // The matrix square root should be calculated using numerically efficient and stable methods such as the Cholesky decomposition
    if (rows_ != cols_) {
      throw std::runtime_error("Matrix must be square.");
    }
    return CholeskyDecomposition();
  }

  double Determinant() {
    if (rows() != cols()) {
      throw std::runtime_error("Matrix must be square.");
    }
    switch (rows()) {
      case 0:
        return 1;
      case 1:
        return m_[0];
      case 2: {
        return m_[0] * m_[3] - m_[1] * m_[2];
      }
      case 3: {
        double a = at(0, 0);
        double b = at(0, 1);
        double c = at(0, 2);
        double d = at(1, 0);
        double e = at(1, 1);
        double f = at(1, 2);
        double g = at(2, 0);
        double h = at(2, 1);
        double i = at(2, 2);
        return a * e * i + b * f * g + c * d * h - a * f * h - b * d * i
            - c * e * g;
      }
      default: {
        int swapCount;
        std::vector<std::size_t> pivot;
        Matrix LU = LUDecompositionByGE(&swapCount, &pivot);
        double result = 1.0;
        for (std::size_t k = 0; k < LU.order(); k++) {
          result *= LU.at(pivot[k], k);
        }
        if (swapCount % 2 == 1) {
          result = -result;
        }
        return result;
      }
    }
  }
  double at(std::size_t row, std::size_t col) const {
    return m_[GetIndex(row, col)];
  }

  void ScaleRow(std::size_t r, double k) {
    for (std::size_t c = 0; c < cols_; c++) {
      m_[GetIndex(r, c)] = k * at(r, c);
    }
  }

  void CombineRow(std::size_t srcRow, std::size_t dstRow, double k) {
    for (std::size_t c = 0; c < cols_; c++) {
      m_[GetIndex(dstRow, c)] = at(dstRow, c) + k * at(srcRow, c);
    }
  }

  Matrix LUDecompositionByGE(int* p_swap_count,
                             std::vector<std::size_t>* p_pivot) {
    if (rows_ != cols_) {
      throw std::runtime_error("Matrix must be square.");
    }
    std::vector<std::size_t>& pivot = *p_pivot;
    *p_swap_count = 0;
    std::size_t n = rows();

    // Make a copy of the matrix.
    // We never change the original matrix because the backing array may be reused.
    std::vector<double> a(m_);
    Matrix m(n, n, a);

    InitPivots(n, p_pivot);
    for (std::size_t k = 0; k < n - 1; k++) {
      double maxValue = m.PartialPivot(p_pivot, k, p_swap_count);
      if (maxValue == 0) {
        throw std::runtime_error("Matrix is singular.");
      }
      double m_kk = m.at(pivot[k], k);
      for (std::size_t i = k + 1; i < n; i++) {
        std::size_t ik = m.GetIndex(pivot[i], k);
        m.m_[ik] /= m_kk;
      }
      for (std::size_t i = k + 1; i < n; i++) {
        double m_ik = m.at(pivot[i], k);
        for (std::size_t j = k + 1; j < n; j++) {
          double m_kj = m.at(pivot[k], j);
          std::size_t ij = m.GetIndex(pivot[i], j);
          m.m_[ij] -= m_ik * m_kj;
        }
      }
    }
    if (m.at(pivot[n - 1], n - 1) == 0) {
      throw std::runtime_error("Matrix is singular.");
    }
    return m;
  }

private:
  std::size_t GetIndex(std::size_t row, std::size_t col) const {
    if (row_based_) return row * cols_ + col;
    else return col * rows_ + row;
  }
  void InitPivots(std::size_t n, std::vector<std::size_t>* p_pivot) {
    std::vector<std::size_t>& pivot = *p_pivot;
    pivot.resize(n);
    for (std::size_t i = 0; i < pivot.size(); i++) {
      pivot[i] = i;
    }
  }
  double PartialPivot(std::vector<std::size_t>* p_pivot, std::size_t k,
                      int* p_swap_count) {
    std::vector<std::size_t>& pivot = *p_pivot;
    int& swap_count = *p_swap_count;
    double maxValue = fabs(at(pivot[k], k));
    std::size_t maxIndex = k;
    for (std::size_t i = k + 1; i < pivot.size(); i++) {
      std::size_t rowIndex = pivot[i];
      double rowValue = fabs(at(rowIndex, k));
      if (rowValue > maxValue) {
        maxValue = rowValue;
        maxIndex = i;
      }
    }
    if (maxIndex != k) {
      std::size_t temp = pivot[maxIndex];
      pivot[maxIndex] = pivot[k];
      pivot[k] = temp;
      swap_count++;
    }
    return maxValue;
  }
  bool IsSymmetric() const {
    if (rows_ != cols_) {
      return false;
    }
    for (std::size_t i = 1; i < rows(); i++) {
      for (std::size_t j = 1; j < i; j++) {
        if (at(i, j) != at(j, i)) {
          return false;
        }
      }
    }
    return true;
  }
  Matrix CholeskyDecomposition() const {
    // returns an upper diagonal matrix
    if (rows_ != cols_) {
      throw std::runtime_error("Matrix must be square.");
    }
    if (!IsSymmetric()) {
      throw std::runtime_error("Matrix must be symmetric.");
    }
    std::size_t n = rows();
    std::vector<double> a(m_);
    for (std::size_t i = 0; i < n; i++) {
      std::size_t ii = GetIndex(i, i);
      for (std::size_t k = 0; k < i; k++) {
        std::size_t ki = GetIndex(k, i);
        a[ii] = a[ii] - a[ki] * a[ki];
      }
      if (a[ii] < 0) {
        throw std::runtime_error("Matrix is not positive definite.");
      }
      a[ii] = sqrt(a[ii]);
      for (std::size_t j = i + 1; j < n; j++) {
        std::size_t ij = GetIndex(i, j);
        for (std::size_t k = 0; k < i; k++) {
          std::size_t ki = GetIndex(k, i);
          std::size_t kj = GetIndex(k, j);
          a[ij] = a[ij] - a[ki] * a[kj];
        }
        if (a[ij] != 0) a[ij] = a[ij] / a[ii];
      }
    }
    // Clear out the lower matrix
    for (std::size_t i = 1; i < n; i++) {
      for (std::size_t j = 0; j < i; j++) {
        std::size_t ij = GetIndex(i, j);
        a[ij] = 0;
      }
    }
    return Matrix(n, n, a);
  }

  std::size_t rows_;
  std::size_t cols_;
  std::vector<double> m_;
  bool row_based_ = true;

  friend Matrix operator*(double scalar, const Matrix& m);
};

}  // namespace data
}  // namespace math
}  // namespace cognitoware

#endif /* COGNITOWARE_MATH_DATA_MATRIX_H_ */
