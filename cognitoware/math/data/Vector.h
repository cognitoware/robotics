/*
 * Vector.h
 *
 *  Copyright (c) 2015, Norman Alan Oursland
 *  All rights reserved.
 */

#ifndef COGNITOWARE_MATH_DATA_VECTOR_H_
#define COGNITOWARE_MATH_DATA_VECTOR_H_

#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace cognitoware {
namespace math {
namespace data {

class Vector {
public:
  Vector() {
  }

  explicit Vector(std::vector<double> a) :
      a_(std::move(a)) {
    a_.resize(a_.size());
  }

  explicit Vector(const Vector& v) :
      a_(v.a_) {
    a_.resize(a_.size());
  }

  Vector(Vector && v) :
      a_(std::move(v.a_)) {
    a_.resize(a_.size());
  }

  Vector(std::size_t order) : a_(order) {
  }

  virtual ~Vector() {
  }

  double Dot(const Vector& v2) const {
    const Vector& v1 = *this;
    double result = 0.0;
    for (std::size_t i = 0; i < v1.order(); i++) {
      result += v1.a_[i] * v2.a_[i];
    }
    return result;

  }

  std::string AsString() const {
    std::ostringstream result;
    result << "{";
    for (std::size_t i = 0; i < a_.size(); i++) {
      if (i > 0) result << ", ";
      result << a_[i];
    }
    result << "}";
    return result.str();
  }

  std::size_t order() const {
    return a_.size();
  }

  void Set(Vector a) {
    if( order() != a.order() ) {
      throw std::runtime_error(
            "Length of array is different from the Vector expected order.");
    }
    a_ = std::move(a.a_);
  }

  double& at(int i) {
    return a_[i];
  }

  double at(int i) const {
    return a_[i];
  }

  Vector operator+(const Vector& v2) const {
    const Vector& v1 = *this;
    if(v1.order() != v2.order()) {
      throw std::runtime_error("Vector sizes are not compatible.");
    }
    Vector result(std::vector<double>(v1.order()));
    std::vector<double>& a = result.a_;
    for (std::size_t i = 0; i < a.size(); i++) {
      a[i] = v1.a_[i] + v2.a_[i];
    }
    return result;
  }

  Vector operator-(const Vector& v2) const {
    const Vector& v1 = *this;
    if(v1.order() != v2.order()) {
      throw std::runtime_error("Vector sizes are not compatible.");
    }
    Vector result(std::vector<double>(v1.order()));
    std::vector<double>& a = result.a_;
    for (std::size_t i = 0; i < a.size(); i++) {
      a[i] = v1.a_[i] - v2.a_[i];
    }
    return result;
  }

  bool operator<(const Vector& v2) const {
    if (this == &v2) return false;
    const Vector& v1 = *this;
    bool result = v1.order() > 0;
    for (std::size_t i = 0; i < v1.order(); i++) {
      result = result && v1.a_[i] < v2.a_[i];
    }
    return result;
  }

  bool operator>(const Vector& v2) const {
    if (this == &v2) return false;
    const Vector& v1 = *this;
    bool result = v1.order() > 0;
    for (std::size_t i = 0; i < v1.order(); i++) {
      result = result && v1.a_[i] > v2.a_[i];
    }
    return result;
  }

  bool operator==(const Vector& v) const {
    if (this == &v) return true;
    return a_ == v.a_;
  }

  Vector& operator=(const Vector& that) {
    this->a_ = that.a_;
    return *this;
  }

  // operator= is implicitly declared as deleted in subclasses
  // because it also defines a move assignment operator.
  void copyAssign(const Vector& v) {
    if (this != &v) {
      a_ = v.a_;
    }
  }

  void moveAssign(Vector&& v) {
    if (this != &v) {
      a_ = std::move(v.a_);
    }
  }

  double& operator[](std::size_t i) {
    return a_[i];
  }

  double operator[](std::size_t i) const {
    return a_[i];
  }

  Vector AliasVector() const { return Vector(a_); }
protected:

private:
  std::vector<double> a_;
};

#define DEFINE_VECTOR1(X) \
class X : public ::cognitoware::math::data::Vector { \
public: \
  X() : Vector(std::vector<double>({ 0.0 })) {} \
  X(double x) : Vector(std::vector<double>({ x })) {} \
  X(const X& v) : Vector(v) {} \
  X(const X&& v) : Vector(v) {} \
  X(::cognitoware::math::data::Vector&& v) : Vector(v) {} \
  X& operator=(const X& that) { Vector::operator=(that); return *this; } \
}

}  // namespace data
}  // namespace math
}  // namespace cognitoware

#endif /* COGNITOWARE_MATH_DATA_VECTOR_H_ */
