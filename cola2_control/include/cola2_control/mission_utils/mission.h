//
// Created by narcis on 6/03/18.
//

#ifndef COLA2_CONTROL_MISSION_H
#define COLA2_CONTROL_MISSION_H

#include <vector>
#include <string>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <algorithm>
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
  std::vector<std::shared_ptr<MissionStep> > mission_;

public:
  Mission();

  ~Mission();

  void loadMission(const std::string& mission_file_name);

  void loadStep(TiXmlHandle hDoc, MissionStep& step);

  void loadManeuverWaypoint(TiXmlHandle hDoc, MissionWaypoint& waypoint);

  void loadManeuverSection(TiXmlHandle hDoc, MissionSection& section);

  void loadManeuverPark(TiXmlHandle hDoc, MissionPark& park);

  void loadAction(TiXmlHandle hDoc, MissionAction& action);

  void loadPosition(TiXmlHandle hDoc, MissionPosition& position);

  void loadTolerance(TiXmlHandle hDoc, MissionTolerance& tolerance);

  void writeMission(std::string mission_file_name);

  void writeMissionStep(TiXmlElement* mission, const MissionStep& step);

  void writeManeuverWaypoint(TiXmlElement* mission, const MissionWaypoint& wp);

  void writeManeuverSection(TiXmlElement* mission, const MissionSection& sec);

  void writeManeuverPark(TiXmlElement* mission, const MissionPark& park);

  void writeAction(TiXmlElement* mission, const MissionAction& action);

  void writeManeuverPosition(TiXmlElement* maneuver, const MissionPosition& p, std::string position_tag);

  void writeManeuverTolerance(TiXmlElement* maneuver, const MissionTolerance& tol);

  void addStep(std::shared_ptr<MissionStep> step);

  std::shared_ptr<MissionStep> getStep(std::size_t i);

  std::size_t size();
};

#endif //COLA2_CONTROL_MISSION_H
