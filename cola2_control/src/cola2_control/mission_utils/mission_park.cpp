//
// Created by narcis on 6/03/18.
//

#include <cola2_control/mission_utils/mission_park.h>

MissionPark::MissionPark() : MissionManeuver(PARK_MANEUVER)
{
}

MissionPark::~MissionPark()
{
}

MissionPark::MissionPark(const MissionPosition& position, const unsigned int t, const MissionTolerance& tolerance)
    : MissionManeuver(PARK_MANEUVER), position_(position), time_(t), tolerance_(tolerance)
{
}

double MissionPark::x()
{
  return position_.getLatitude();
}

double MissionPark::y()
{
  return position_.getLongitude();
}

double MissionPark::z()
{
  return position_.getZ();
}

MissionPosition MissionPark::getPosition() const
{
  return position_;
}

unsigned int MissionPark::getTime() const
{
  return time_;
}

MissionTolerance MissionPark::getTolerance() const
{
  return tolerance_;
}

void MissionPark::setPosition(const MissionPosition& position)
{
  position_ = position;
}

void MissionPark::setTime(const unsigned int t)
{
  time_ = t;
}

void MissionPark::setTolerance(const MissionTolerance& tolerance)
{
  tolerance_ = tolerance;
}
