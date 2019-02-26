//
// Created by narcis on 6/03/18.
//

#include <cola2_control/mission_utils/mission_step.h>

MissionStep::MissionStep()
{
}

MissionStep::~MissionStep()
{
}

std::shared_ptr<MissionManeuver> MissionStep::getManeuverPtr() const
{
  return maneuver_;
}

std::vector<MissionAction> MissionStep::getActions() const
{
  return actions_;
}

void MissionStep::setManeuverPtr(std::shared_ptr<MissionManeuver> maneuver)
{
  maneuver_ = maneuver;
}

void MissionStep::addAction(const MissionAction& action)
{
  actions_.push_back(action);
}
