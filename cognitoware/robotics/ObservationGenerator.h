/*
 * ObservationGenerator.h
 *
 *  Copyright (c) 2015, Norman Alan Oursland
 *  All rights reserved.
 */

#ifndef ROBOTICS_OBSERVATIONGENERATOR_H_
#define ROBOTICS_OBSERVATIONGENERATOR_H_

#include <random>

namespace cognitoware {
namespace robotics {

template<typename T>
class ObservationGenerator {
public:
  virtual ~ObservationGenerator() {
  }

  virtual void CreateObservation(std::default_random_engine* generator,
                                 T* result_out) = 0;
};

}  // namespace robotics
}  // namespace cognitoware

#endif /* ROBOTICS_OBSERVATIONGENERATOR_H_ */
