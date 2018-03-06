//
// Created by narcis on 6/03/18.
//

#ifndef COLA2_CONTROL_MISSION_H
#define COLA2_CONTROL_MISSION_H

#include <vector>
#include <string>
#include <iostream>
#include "tinyxml.h"
#include <cola2_control/mission_utils/mission_maneuver.h>
#include <cola2_control/mission_utils/mission_action.h>
#include <cola2_control/mission_utils/mission_configuration.h>
#include <cola2_control/mission_utils/mission_park.h>
#include <cola2_control/mission_utils/mission_position.h>
#include <cola2_control/mission_utils/mission_tolerance.h>
#include <cola2_control/mission_utils/mission_step.h>
#include <cola2_control/mission_utils/mission_section.h>
#include <cola2_control/mission_utils/mission_waypoint.h>

/**
 * \brief Mission class from mission types.
 */
class Mission
{
private:
  unsigned int step_size_;
  std::vector<MissionStep*> mission_;

public:
  Mission();

  ~Mission();

  MissionStep* getStep(unsigned int i);

  unsigned int size();

  void show();

  std::string to_string(const double value) const;

  void addStep(MissionStep* step);

  int loadAction(TiXmlHandle hDoc, MissionAction& action);

  bool loadPosition(TiXmlHandle hDoc, MissionPosition& position);

  bool loadTolerance(TiXmlHandle hDoc, MissionTolerance& tolerance);

  bool loadManeuverWaypoint(TiXmlHandle hDoc, MissionWaypoint& waypoint);

  bool loadManeuverSection(TiXmlHandle hDoc, MissionSection& section);

  bool loadManeuverPark(TiXmlHandle hDoc, MissionPark& park);

  int loadStep(TiXmlHandle hDoc, MissionStep& step);

  int loadMission(const std::string mission_file_name);

  int writeAction(TiXmlElement* mission, MissionAction& action);

  int writeManeuverPosition(TiXmlElement* maneuver, const MissionPosition& p, std::string position_tag);

  int writeManeuverTolerance(TiXmlElement* maneuver, const MissionTolerance& tol);

  int writeManeuverWaypoint(TiXmlElement* mission, const MissionWaypoint& wp);

  int writeManeuverSection(TiXmlElement* mission, const MissionSection& sec);

  int writeManeuverPark(TiXmlElement* mission, const MissionPark& park);

  int writeMissionStep(TiXmlElement* mission, const MissionStep& step);

  int writeMission(std::string mission_file_name);
};

#endif //COLA2_CONTROL_MISSION_H
