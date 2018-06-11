//
// Created by narcis on 6/03/18.
//

#include <cola2_control/mission_utils/mission_maneuver.h>

MissionManeuver::MissionManeuver(const unsigned int type) : maneuver_type_(type)
{
}

MissionManeuver::~MissionManeuver()
{
}

unsigned int MissionManeuver::getManeuverType()
{
  return maneuver_type_;
}
