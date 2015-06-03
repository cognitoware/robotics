/*
 * ActionModel.h
 *
 *  Copyright (c) 2015, Norman Alan Oursland
 *  All rights reserved.
 */

#ifndef COGNITOWARE_ROBOTICS_STATE_ESTIMATION_ACTIONMODEL_H_
#define COGNITOWARE_ROBOTICS_STATE_ESTIMATION_ACTIONMODEL_H_

namespace cognitoware {
namespace robotics {
namespace state_estimation {

template<typename U, typename X>
class ActionModel {
public:
  ActionModel() {}
  virtual ~ActionModel() {}
};

}  // namespace state_estimation
}  // namespace robotics
}  // namespace cognitoware

#endif /* COGNITOWARE_ROBOTICS_STATE_ESTIMATION_ACTIONMODEL_H_ */
