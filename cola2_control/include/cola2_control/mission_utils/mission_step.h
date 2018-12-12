//
// Created by narcis on 6/03/18.
//

#ifndef COLA2_CONTROL_MISSION_STEP_H
#define COLA2_CONTROL_MISSION_STEP_H

#include <vector>
#include <iostream>
#include <memory>
#include <cola2_control/mission_utils/mission_maneuver.h>
#include <cola2_control/mission_utils/mission_action.h>
#include <memory>

/**
 * \brief MissionStep class from mission types.
 */
class MissionStep
{
private:
  std::shared_ptr<MissionManeuver> maneuver_;
  std::vector<MissionAction> actions_;

public:
  MissionStep();

  ~MissionStep();

  std::shared_ptr<MissionManeuver> getManeuverPtr() const;

  std::vector<MissionAction> getActions() const;

  void setManeuverPtr(std::shared_ptr<MissionManeuver> maneuver);

  void addAction(const MissionAction& action);
};

#endif //COLA2_CONTROL_MISSION_STEP_H
