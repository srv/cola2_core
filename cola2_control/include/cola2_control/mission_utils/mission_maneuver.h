//
// Created by narcis on 6/03/18.
//

#ifndef COLA2_CONTROL_MISSION_MANEUVER_H
#define COLA2_CONTROL_MISSION_MANEUVER_H

#include <vector>
#include <string>
#include <iostream>

const unsigned int WAYPOINT_MANEUVER = 0;
const unsigned int SECTION_MANEUVER = 1;
const unsigned int PARK_MANEUVER = 2;
const unsigned int MISSION_MANEUVER = 0;
const unsigned int MISSION_CONFIGURATION = 1;
const unsigned int MISSION_ACTION = 2;

/**
 * \brief MissionManeuver class from mission types.
 */
class MissionManeuver
{
private:
  unsigned int maneuver_type_;

public:
  MissionManeuver(const unsigned int type);
  ~MissionManeuver();

//  friend std::ostream& operator<<(std::ostream& stream, const MissionManeuver& mm)
//  {
//    stream << mm;
//  }

  unsigned int getManeuverType();

  virtual double x() = 0;

  virtual double y() = 0;

  virtual double z() = 0;
};

#endif //COLA2_CONTROL_MISSION_MANEUVER_H
