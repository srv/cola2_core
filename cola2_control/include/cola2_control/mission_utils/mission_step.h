//
// Created by narcis on 6/03/18.
//

#ifndef COLA2_CONTROL_MISSION_STEP_H
#define COLA2_CONTROL_MISSION_STEP_H

#include <vector>
#include <string>
#include <iostream>
#include <cola2_control/mission_utils/mission_maneuver.h>
#include <cola2_control/mission_utils/mission_action.h>
#include <memory>

/**
 * \brief MissionStep class from mission types.
 */
class MissionStep
{
private:
  unsigned int step_id_;
  MissionManeuver* maneuver_;
  std::vector<MissionAction> actions_;

public:
  MissionStep();

  ~MissionStep();

  MissionManeuver* getManeuverPtr() const;

  std::vector<MissionAction> getActions() const;

  void setManeuverPtr(MissionManeuver* maneuver);

  void addAction(MissionAction action);

  unsigned int getStepId();

  void incStepId();
};

#endif //COLA2_CONTROL_MISSION_STEP_H
