//
// Created by narcis on 6/03/18.
//

#include <cola2_control/mission_utils/mission_waypoint.h>

MissionWaypoint::MissionWaypoint() : MissionManeuver(WAYPOINT_MANEUVER)
{
}

MissionWaypoint::~MissionWaypoint()
{
}

MissionWaypoint::MissionWaypoint(MissionPosition position, double speed, MissionTolerance tolerance)
    : MissionManeuver(WAYPOINT_MANEUVER), position_(position), speed_(speed), tolerance_(tolerance)
{
}

double MissionWaypoint::x()
{
  return position_.getLatitude();
}

double MissionWaypoint::y()
{
  return position_.getLongitude();
}

double MissionWaypoint::z()
{
  return position_.getZ();
}

MissionPosition MissionWaypoint::getPosition() const
{
  return position_;
}

double MissionWaypoint::getSpeed() const
{
  return speed_;
}

MissionTolerance MissionWaypoint::getTolerance() const
{
  return tolerance_;
}

void MissionWaypoint::setPosition(const MissionPosition position)
{
  position_ = position;
}

void MissionWaypoint::setSpeed(const double speed)
{
  speed_ = speed;
}

void MissionWaypoint::setTolerance(const MissionTolerance tolerance)
{
  tolerance_ = tolerance;
}

