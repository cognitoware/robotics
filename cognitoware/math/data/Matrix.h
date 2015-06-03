/*
 * Matrix.h
 *
 *  Copyright (c) 2015, Norman Alan Oursland
 *  All rights reserved.
 */

#ifndef COGNITOWARE_MATH_DATA_MATRIX_H_
#define COGNITOWARE_MATH_DATA_MATRIX_H_

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
      rows_(rows), cols_(cols), m_(rows*cols), row_based_(true) {
  }

  Matrix(std::size_t rows, std::size_t cols, std::vector<double> values) :
      rows_(rows), cols_(cols), m_(values), row_based_(true) {
  }

  Matrix(std::size_t rows, std::size_t cols, std::vector<double> values,
         bool row_based) :
      rows_(rows), cols_(cols), m_(values), row_based_(row_based) {
  }

  explicit Matrix(const Matrix& that) :
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
  Matrix Transpose() const {
    return Matrix(cols_, rows_, m_, !row_based_);
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

  double at(std::size_t row, std::size_t col) const {
    return m_[GetIndex(row, col)];
  }

private:
  std::size_t GetIndex(std::size_t row, std::size_t col) const {
    if (row_based_) return row * cols_ + col;
    else return col * rows_ + row;
  }

  std::size_t rows_;
  std::size_t cols_;
  std::vector<double> m_;
  bool row_based_ = true;

};

}  // namespace data
}  // namespace math
}  // namespace cognitoware

#endif /* COGNITOWARE_MATH_DATA_MATRIX_H_ */
