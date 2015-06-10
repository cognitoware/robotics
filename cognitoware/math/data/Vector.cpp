/*
 * Vector.cpp
 *
 *  Copyright (c) 2015, Norman Alan Oursland
 *  All rights reserved.
 */

#include "cognitoware/math/data/Vector.h"

namespace cognitoware {
namespace math {
namespace data {

Vector::Vector() {
}

Vector::Vector(std::vector<double> a) :
    a_(std::move(a)) {
}

Vector::Vector(const Vector& v) :
    a_(v.a_) {
}

Vector::Vector(Vector && v) :
    a_(std::move(v.a_)) {
}

Vector::Vector(std::size_t order) :
    a_(order) {
}

Vector::~Vector() {
}

double Vector::Dot(const Vector& v2) const {
  const Vector& v1 = *this;
  double result = 0.0;
  for (std::size_t i = 0; i < v1.order(); i++) {
    result += v1.a_[i] * v2.a_[i];
  }
  return result;

}

std::size_t Vector::order() const {
  return a_.size();
}

void Vector::Set(Vector a) {
  if (order() != a.order()) {
    throw std::runtime_error(
        "Length of array is different from the Vector expected order.");
  }
  a_ = std::move(a.a_);
}

double& Vector::at(int i) {
  return a_[i];
}

double Vector::at(int i) const {
  return a_[i];
}

Vector Vector::operator+(const Vector& v2) const {
  const Vector& v1 = *this;
  if (v1.order() != v2.order()) {
    throw std::runtime_error("Vector sizes are not compatible.");
  }
  Vector result(std::vector<double>(v1.order()));
  std::vector<double>& a = result.a_;
  for (std::size_t i = 0; i < a.size(); i++) {
    a[i] = v1.a_[i] + v2.a_[i];
  }
  return result;
}

Vector Vector::operator-(const Vector& v2) const {
  const Vector& v1 = *this;
  if (v1.order() != v2.order()) {
    throw std::runtime_error("Vector sizes are not compatible.");
  }
  Vector result(v1.order());
  std::vector<double>& a = result.a_;
  for (std::size_t i = 0; i < a.size(); i++) {
    a[i] = v1.a_[i] - v2.a_[i];
  }
  return result;
}

bool Vector::operator<(const Vector& v2) const {
  if (this == &v2) return false;
  const Vector& v1 = *this;
  bool result = v1.order() > 0;
  for (std::size_t i = 0; i < v1.order(); i++) {
    result = result && v1.a_[i] < v2.a_[i];
  }
  return result;
}

bool Vector::operator>(const Vector& v2) const {
  if (this == &v2) return false;
  const Vector& v1 = *this;
  bool result = v1.order() > 0;
  for (std::size_t i = 0; i < v1.order(); i++) {
    result = result && v1.a_[i] > v2.a_[i];
  }
  return result;
}

bool Vector::operator==(const Vector& v) const {
  if (this == &v) return true;
  return a_ == v.a_;
}

Vector& Vector::operator=(const Vector& that) {
  this->a_ = that.a_;
  return *this;
}

// operator= is implicitly declared as deleted in subclasses
// because it also defines a move assignment operator.
void Vector::copyAssign(const Vector& v) {
  if (this != &v) {
    a_ = v.a_;
  }
}

void Vector::moveAssign(Vector&& v) {
  if (this != &v) {
    a_ = std::move(v.a_);
  }
}

double& Vector::operator[](std::size_t i) {
  return a_[i];
}

double Vector::operator[](std::size_t i) const {
  return a_[i];
}

Matrix Vector::Cross(const Vector& that) const {
  std::vector<double> m(order() * that.order());
  int k = 0;
  for (std::size_t i = 0; i < order(); i++) {
    double a = a_[i];
    for (std::size_t j = 0; j < that.order(); j++) {
      double b = that[j];
      m[k] = a * b;
      k++;
    }
  }
  return Matrix(order(), that.order(), m);
}

Vector Vector::AliasVector() const {
  return Vector(a_);
}

std::ostream& operator<<(std::ostream& os, const Vector& v) {
  os << v.order() << ":{";
  for (std::size_t i = 0; i < v.order(); i++) {
    if (i > 0) os << ", ";
    os << v[i];
  }
  os << "}";
  return os;
}

}  // namespace data
}  // namespace math
}  // namespace cognitoware
