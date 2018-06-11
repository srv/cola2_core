//
// Created by narcis on 6/03/18.
//

#ifndef COLA2_CONTROL_MISSION_WAYPOINT_H
#define COLA2_CONTROL_MISSION_WAYPOINT_H

#include <vector>
#include <string>
#include <iostream>
#include <cola2_control/mission_utils/mission_position.h>
#include <cola2_control/mission_utils/mission_tolerance.h>
#include <cola2_control/mission_utils/mission_maneuver.h>

/**
 * \brief MissionWaypoint class from mission types.
 */

class MissionWaypoint : public MissionManeuver
{
private:
  MissionPosition position_;
  double speed_;
  MissionTolerance tolerance_;

public:
  MissionWaypoint();

  ~MissionWaypoint();


  MissionWaypoint(MissionPosition position_, double speed_, MissionTolerance tolerance_);

//  friend std::ostream& operator<<(std::ostream& stream, const MissionWaypoint& wp)
//  {
//    stream << "Waypoint -> " << wp.getPosition() << " at " << wp.getSpeed() << "m/s with tolerance " << wp.getTolerance();
//  }

  MissionPosition getPosition() const;

  double getSpeed() const;

  MissionTolerance getTolerance() const;

  void setPosition(const MissionPosition position);

  void setSpeed(const double speed);

  void setTolerance(const MissionTolerance tolerance);

  double x();

  double y();

  double z();
};

#endif //COLA2_CONTROL_MISSION_WAYPOINT_H
