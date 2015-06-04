/*
 * PiecewiseLinearFunction.cpp
 *
 *  Copyright (c) 2015, Norman Alan Oursland
 *  All rights reserved.
 */

#include <cognitoware/math/data/PiecewiseLinearFunction.h>

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>

namespace cognitoware {
namespace math {
namespace data {

PiecewiseLinearFunction::PiecewiseLinearFunction(PiecewiseLinearFunction::Fn fn,
                                                 double min, double max, int n,
                                                 double upperError,
                                                 double lowerError) {
  std::list<std::pair<double, double>> pts = BuildFn(fn, min, max, n,
      upperError, lowerError);
  domain_.resize(pts.size());
  range_.resize(pts.size());
  int i = 0;
  for (auto& pt : pts) {
    domain_[i] = pt.first;
    range_[i] = pt.second;
    i++;
  }
}

PiecewiseLinearFunction::PiecewiseLinearFunction(
    const PiecewiseLinearFunction&& that) :
    domain_(std::move(that.domain_)), range_(std::move(that.range_)) {

}
PiecewiseLinearFunction& PiecewiseLinearFunction::operator=(
    PiecewiseLinearFunction&& that) {
  domain_ = std::move(that.domain_);
  range_ = std::move(that.range_);
  return *this;
}

double PiecewiseLinearFunction::Evaluate(double x) const {
  auto found_iter = std::lower_bound(domain_.begin(), domain_.end(), x);
  std::size_t index = found_iter - domain_.begin();
  if (index < domain_.size() && *found_iter == x) {
    return range_[index];
  }
  if (index >= domain_.size()) {
    return range_[range_.size() - 1];
  }
  if (index <= 0) {
    return range_[0];
  }
  double x0 = domain_[index - 1];
  double x1 = domain_[index];
  double p = (x - x0) / (x1 - x0);
  double y0 = range_[index - 1];
  double y1 = range_[index];
  return y0 + p * (y1 - y0);
}

PiecewiseLinearFunction PiecewiseLinearFunction::PdfToCdf() {
  PiecewiseLinearFunction cdf;
  double sum = 0.0;
  std::size_t n = range_.size();
  cdf.range_.resize(n);
  cdf.domain_.resize(n);
  // assume the probabilities outside of the range_. are zero

  cdf.domain_[0] = 0.0;
  cdf.range_[0] = domain_[0];

  for (std::size_t i = 1; i < domain_.size(); i++) {
    double x = domain_[i];
    double dx = (x - domain_[i - 1]);
    double avgY = (range_[i] + range_[i - 1]) / 2;
    sum += dx * avgY;
    cdf.domain_[i] = sum;
    cdf.range_[i] = x;
  }

  for (std::size_t i = 0; i < domain_.size(); i++) {
    cdf.domain_[i] /= sum;
  }
  return cdf;
}

/* static */
std::list<std::pair<double, double>> PiecewiseLinearFunction::BuildFn(
    PiecewiseLinearFunction::Fn fn, double min, double max, int n,
    double upperError, double lowerError) {
  std::list<std::pair<double, double>> pts;
  for (int i = 0; i <= n; i++) {
    double p = ((double) i / n);
    double x = min * (1 - p) + max * p;
    double y = fn(x);
    pts.push_back(std::pair<double, double>(x, y));
  }
  bool changed = false;
  do {
    changed = false;
    changed = PruneResults(lowerError, &pts) || changed;
    changed = ExpandResults(fn, upperError, &pts) || changed;
  } while (changed);
  return pts;
}

/* static */
bool PiecewiseLinearFunction::ExpandResults(
    PiecewiseLinearFunction::Fn fn, double upperBound,
    std::list<std::pair<double, double>>* p_pts) {
  double maxError = 0;
  bool result = false;
  auto current = p_pts->begin();
  auto next = std::next(current, 1);
  while (p_pts->end() != current && p_pts->end() != next) {
    double x0 = current->first;
    double x1 = next->first;
    double y0 = current->second;
    double y1 = next->second;
    double midx = (x0 + x1) / 2;
    double midy = (y0 + y1) / 2;
    double y = fn(midx);
    double currentArea = (y0 + y1) * (x1 - x0) / 2;
    double proposeArea = (y0 + midy) * (midx - x0) / 2
        + (midy + y1) * (x1 - midx) / 2;
    double err = fabs(currentArea - proposeArea);
    if (err > upperBound) {
      p_pts->insert(next, std::pair<double, double>(midx, y));
      ++current;
      result = true;
    } else {
      ++current;
      ++next;
    }
    maxError = fmax(err, maxError);
  }
  return result;
}

/* static */
bool PiecewiseLinearFunction::PruneResults(double lowerBound,
                  std::list<std::pair<double, double>>* p_pts) {
  double minError = std::numeric_limits<double>::max();
  bool result = false;
  auto current = p_pts->begin();
  auto middle = std::next(current, 1);
  auto next = std::next(middle, 1);

  while (p_pts->end() != current && p_pts->end() != middle
      && p_pts->end() != next) {
    double x0 = current->first;
    double x1 = middle->first;
    double x2 = next->first;
    double y0 = current->second;
    double y1 = middle->second;
    double y2 = next->second;
//    double p = (x1 - x0) / (x2 - x0);
//    double yEst = y0 + p * (y2 - y0);

    double currentArea = (y0 + y2) * (x2 - x0) / 2;
    double proposeArea = (y0 + y1) * (x1 - x0) / 2 + (y1 + y2) * (x2 - x1) / 2;
    double err = fabs(currentArea - proposeArea);

    if (err < lowerBound) {
      p_pts->erase(middle);
      middle = next;
      ++next;
      result = true;
    } else {
      current = middle;
      middle = next;
      ++next;
    }
    minError = fmin(err, minError);
  }
  return result;
}

}  // namespace data
}  // namespace math
}  // namespace cognitoware
