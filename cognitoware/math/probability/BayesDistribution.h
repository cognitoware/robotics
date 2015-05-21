/*
 * BayesDistribution.h
 *
 *  Created on: Jan 19, 2015
 *      Author: Alan Oursland
 */

#ifndef MATH_PROBABILITY_BAYESDISTRIBUTION_H_
#define MATH_PROBABILITY_BAYESDISTRIBUTION_H_

#include "cognitoware/math/probability/RandomConditional.h"
#include "cognitoware/math/probability/RandomDistribution.h"

#include <memory>
#include <vector>

namespace cognitoware {
namespace math {
namespace probability {

template<typename X, typename Y>
class BayesDistribution : public RandomDistribution<X> {
public:
  BayesDistribution(std::shared_ptr<const RandomDistribution<X>> prior,
                    std::shared_ptr<const RandomConditional<Y, X>> sensorModel,
                    Y observation)
      : prior_(prior), sensorModel_(sensorModel), data_(observation) {

  }
  virtual ~BayesDistribution() {
  }

  double ProbabilityOf(X x) const override {
    double pyx = sensorModel_->ConditionalProbabilityOf(data_, x);
    double px = prior_->ProbabilityOf(x);
    double py = normalizer_;
    return pyx * px / py;
  }

  X Sample(std::default_random_engine*) const override {
    throw std::runtime_error("Not Implemented");
  }

  std::shared_ptr<BayesDistribution<X, Y>> ChainObservation(Y y) {
    return std::make_shared<BayesDistribution<X, Y>>(this, sensorModel_, y);
  }

  double EstimateNormalizer(const std::vector<X>& domain, double dx) {
    double sum = 0.0;
    for (X x : domain) {
      sum += sensorModel_.ConditionalProbabilityOf(data_, x)
          * prior_.ProbabilityOf(x);
    }
    normalizer_ = sum * dx;
    return normalizer_;
  }

  // TODO: Redesign alias
//  public override T AliasAs<T>() {
//    T result = base.AliasAs<T>();
//    if( result == null ) {
//      result = SensorModel.LikelihoodOf(RandomDistribution<X>) as T;
//    }
//    return result;
//  }

  const RandomConditional<Y, X>& sensor_model() const {
    return *sensorModel_;
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
  std::shared_ptr<const RandomConditional<Y, X>> sensorModel_;
  Y data_;
  double normalizer_ = 1.0;
};

}  // namespace probability
}  // namespace math
}  // namespace cognitoware

#endif /* MATH_PROBABILITY_BAYESDISTRIBUTION_H_ */
