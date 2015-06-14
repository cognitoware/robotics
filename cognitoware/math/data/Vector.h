/*
 * Vector.h
 *
 *  Copyright (c) 2015, Norman Alan Oursland
 *  All rights reserved.
 */

#ifndef COGNITOWARE_MATH_DATA_VECTOR_H_
#define COGNITOWARE_MATH_DATA_VECTOR_H_

#include "cognitoware/math/data/Matrix.h"

#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace cognitoware {
namespace math {
namespace data {

class Matrix;

class Vector {
public:
  Vector();
  explicit Vector(std::vector<double> a);
  explicit Vector(const Vector& v);
  Vector(Vector && v);
  explicit Vector(std::size_t order);
  virtual ~Vector();
  std::size_t order() const;
  double& at(int i);
  double at(int i) const;
  double& operator[](std::size_t i);
  double operator[](std::size_t i) const;
  Matrix Cross(const Vector& that) const;
  Vector& operator=(const Vector& that);
  virtual void Set(Vector a);
  void copyAssign(const Vector& v);
  void moveAssign(Vector&& v);
  Vector AliasVector() const;
  double Dot(const Vector& v2) const;
  Vector operator+(const Vector& v2) const;
  Vector operator-(const Vector& v2) const;
  bool operator<(const Vector& v2) const;
  bool operator>(const Vector& v2) const;
  bool operator==(const Vector& v) const;
protected:

private:
  std::vector<double> a_;
};

std::ostream& operator<<(std::ostream& os, const Vector& v);


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
