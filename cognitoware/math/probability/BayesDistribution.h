/*
 * BayesDistribution.h
 *
 *  Copyright (c) 2015, Norman Alan Oursland
 *  All rights reserved.
 */

#ifndef COGNITOWARE_MATH_PROBABILITY_BAYESDISTRIBUTION_H_
#define COGNITOWARE_MATH_PROBABILITY_BAYESDISTRIBUTION_H_

#include "cognitoware/math/probability/RandomConditional.h"
#include "cognitoware/math/probability/RandomDistribution.h"

#include <memory>
#include <vector>

namespace cognitoware {
namespace math {
namespace probability {

template<typename X, typename Y>
class BayesDistribution : public RandomDistribution<X>,
    public std::enable_shared_from_this<BayesDistribution<X, Y>> {
public:
  BayesDistribution(std::shared_ptr<const RandomDistribution<X>> prior,
                    std::shared_ptr<const RandomConditional<Y, X>> sensor_model,
                    Y observation) :
      prior_(prior), sensor_model_(sensor_model), data_(std::move(observation)) {

  }

  virtual ~BayesDistribution() {
  }

  double ProbabilityOf(const X& x) const override {
    double pyx = sensor_model_->ConditionalProbabilityOf(data_, x);
    double px = prior_->ProbabilityOf(x);
    double py = normalizer_;
    return pyx * px / py;
  }

  X Sample(std::default_random_engine*) const override {
    throw std::runtime_error("Not Implemented");
  }

  std::shared_ptr<BayesDistribution<X, Y>> ChainObservation(Y y) {
    return std::make_shared<BayesDistribution<X, Y>>(this->shared_from_this(),
        sensor_model_, std::move(y));
  }

  double EstimateNormalizer(const std::vector<X>& domain, double dx) {
    double sum = 0.0;
    for (const X& x : domain) {
      double pz_given_x = sensor_model_->ConditionalProbabilityOf(data_, x);
      double px = prior_->ProbabilityOf(x);
      sum += pz_given_x * px;
    }
    normalizer_ = sum * dx;
    return normalizer_;
  }

  const RandomConditional<Y, X>& sensor_model() const {
    return *sensor_model_;
  }

  const Y& data() const {
    return data_;
  }

  double& normalizer() {
    return normalizer_;
  }

  double normalizer() const {
    return normalizer_;
  }

private:
  std::shared_ptr<const RandomDistribution<X>> prior_;
  std::shared_ptr<const RandomConditional<Y, X>> sensor_model_;
  Y data_;
  double normalizer_ = 1.0;
};

}  // namespace probability
}  // namespace math
}  // namespace cognitoware

#endif /* COGNITOWARE_MATH_PROBABILITY_BAYESDISTRIBUTION_H_ */
