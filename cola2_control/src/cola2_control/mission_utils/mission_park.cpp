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

MissionPark::MissionPark(MissionPosition position, unsigned int time, MissionTolerance tolerance)
    : MissionManeuver(PARK_MANEUVER), position_(position), time_(time), tolerance_(tolerance)
{
}

void MissionPark::show()
{
  std::cout << *this;
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

void MissionPark::setPosition(const MissionPosition position)
{
  position_ = position;
}

void MissionPark::setTime(const unsigned int time)
{
  time_ = time;
}

void MissionPark::setTolerance(const MissionTolerance tolerance)
{
  tolerance_ = tolerance;
}
