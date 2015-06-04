/*
 * VectorRange.h
 *
 *  Copyright (c) 2015, Norman Alan Oursland
 *  All rights reserved.
 */

#ifndef COGNITOWARE_MATH_DATA_VECTORRANGE_H_
#define COGNITOWARE_MATH_DATA_VECTORRANGE_H_

#include <cmath>
#include <random>
#include <utility>

namespace cognitoware {
namespace math {
namespace data {

template <typename X>
class VectorRange {
public:
  VectorRange(X min, X max)
      : min_(std::move(min)), max_(std::move(max)) {
  }
  virtual ~VectorRange() {
  }

  double area() const {
    Vector dx = max_ - min_;
    double result = 1.0;
    for (unsigned int i = 0; i < dx.order(); i++)
      result *= dx[i];
    return fabs(result);
  }

  bool Contains(const X& x) const {
    return x > min_ && x < max_;
  }

  X Sample(std::default_random_engine* generator) const {
    X result;
    for (unsigned int i = 0; i < result.order(); i++) {
      std::uniform_real_distribution<double> random(min_[i], max_[i]);
      result[i] = random(*generator);
    }
    return result;
  }

private:
  X min_;
  X max_;
};

}  // namespace data
}  // namespace math
}  // namespace cognitoware

#endif /* COGNITOWARE_MATH_DATA_VECTORRANGE_H_ */
