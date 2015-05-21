/*
 * Vector.h
 *
 *  Copyright (c) 2015, Norman Alan Oursland
 *  All rights reserved.
 */

#ifndef COGNITOWARE_MATH_DATA_VECTOR_H_
#define COGNITOWARE_MATH_DATA_VECTOR_H_

#include <type_traits>
#include <utility>
#include <vector>
#include <sstream>
#include <string>

namespace cognitoware {
namespace math {
namespace data {

template<class SUBTYPE, std::size_t SIZE>
class Vector {
public:
  static constexpr std::size_t Order = SIZE;

//  Matrix Cross(const SUBTYPE& v) const {
//    std::vector<double> m(order() * v.order());
//    int k = 0;
//    for (unsigned int i = 0; i < order(); i++) {
//      double a = at(i);
//      for (unsigned int j = 0; j < v.order(); j++) {
//        double b = v[j];
//        m[k] = a * b;
//        k++;
//      }
//    }
//    return Matrix(order(), v.order(), m);
//  }

  double Dot(const SUBTYPE& v2) const {
    const Vector<SUBTYPE, SIZE>& v1 = *this;
    double result = 0.0;
    for (unsigned int i = 0; i < v1.order(); i++) {
      result += v1.a_[i] * v2.a_[i];
    }
    return result;

  }

  std::string AsString() const {
    std::ostringstream result;
    result << "{";
    for (unsigned int i = 0; i < a_.size(); i++) {
      if (i > 0) result << ", ";
      result << a_[i];
    }
    result << "}";
    return result.str();
  }

  std::size_t order() const {
    return SIZE;
  }
  double& at(int i) {
    return a_[i];
  }
  double at(int i) const {
    return a_[i];
  }

//  SUBTYPE operator*(const Matrix& m) const {
//    const SUBTYPE& v = *this;
//    if (v.order() != m.rows())
//      throw std::invalid_argument("Order/row mismatch.");
//    std::vector<double> result(v.order());
//    for (unsigned int col = 0; col < m.columns(); col++) {
//      for (unsigned int row = 0; row < m.rows(); row++) {
//        result[row] += v[row] * m.at(row, col);
//      }
//    }
//    return SUBTYPE(result);
//  }
  SUBTYPE operator+(const SUBTYPE& v2) const {
    const Vector<SUBTYPE, SIZE>& v1 = *this;
    SUBTYPE result;
    std::vector<double>& a = result.a_;
    for (unsigned int i = 0; i < a.size(); i++) {
      a[i] = v1.a_[i] + v2.a_[i];
    }
    return result;
  }
  SUBTYPE operator-(const SUBTYPE& v2) const {
    const Vector<SUBTYPE, SIZE>& v1 = *this;
    SUBTYPE result;
    std::vector<double>& a = result.a_;
    for (unsigned int i = 0; i < a.size(); i++) {
      a[i] = v1.a_[i] - v2.a_[i];
    }
    return result;
  }
  bool operator<(const SUBTYPE& v2) const {
    if (this == &v2) return false;
    const Vector<SUBTYPE, SIZE>& v1 = *this;
    bool result = v1.order() > 0;
    for (unsigned int i = 0; i < v1.order(); i++) {
      result = result && v1.a_[i] < v2.a_[i];
    }
    return result;
  }
  bool operator>(const SUBTYPE& v2) const {
    if (this == &v2) return false;
    const Vector<SUBTYPE, SIZE>& v1 = *this;
    bool result = v1.order() > 0;
    for (unsigned int i = 0; i < v1.order(); i++) {
      result = result && v1.a_[i] > v2.a_[i];
    }
    return result;
  }
  bool operator==(const SUBTYPE& v) const {
    if (this == &v) return true;
    return a_ == v.a_;
  }
  // operator= is implicitly declared as deleted in subclasses
  // because it also defines a move assignment operator.
  void copyAssign(const SUBTYPE& v) {
    if (this != &v) {
      a_ = v.a_;
    }
  }
  void moveAssign(SUBTYPE&& v) {
    if (this != &v) {
      a_ = std::move(v.a_);
    }
  }
  double& operator[](int i) {
    return a_[i];
  }
  double operator[](int i) const {
    return a_[i];
  }

protected:
  Vector() {
    static_assert(std::is_base_of<Vector<SUBTYPE, SIZE>, SUBTYPE>::value, "SUBTYPE must derive from Vector");
    a_.resize(SIZE);
  }

  explicit Vector(std::vector<double> a)
      : a_(std::move(a)) {
    a_.resize(SIZE);
  }

  explicit Vector(const Vector<SUBTYPE, SIZE>& v)
      : a_(v.a_) {
    a_.resize(SIZE);
  }

  virtual ~Vector() {
  }

  Vector(Vector<SUBTYPE, SIZE> && v)
      : a_(std::move(v.a_)) {
    a_.resize(SIZE);
  }


private:
  std::vector<double> a_;
};

#define DEFINE_VECTOR1(X) \
class X : public Vector<X, 1> { \
public: \
  X() : Vector( { 0.0 }) {} \
  X(double x) : Vector( { x }) {} \
  explicit X(const X& v) : Vector(v) {} \
  X(const X&& v) : Vector(v) {} \
}



}  // namespace data
}  // namespace math
}  // namespace cognitoware

#endif /* MATH_DATA_VECTOR_H_ */
