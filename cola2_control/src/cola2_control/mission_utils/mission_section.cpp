//
// Created by narcis on 6/03/18.
//

#include <cola2_control/mission_utils/mission_section.h>

MissionSection::MissionSection() : MissionManeuver(SECTION_MANEUVER)
{
}

MissionSection::MissionSection(const MissionPosition& initial_position, const MissionPosition& final_position,
                               const double speed, const MissionTolerance& tolerance)
    : MissionManeuver(SECTION_MANEUVER)
    , initial_position_(initial_position)
    , final_position_(final_position)
    , speed_(speed)
    , tolerance_(tolerance)
{
}

MissionSection::~MissionSection()
{
}

MissionPosition MissionSection::getInitialPosition() const
{
  return initial_position_;
}

MissionPosition MissionSection::getFinalPosition() const
{
  return final_position_;
}

double MissionSection::getSpeed() const
{
  return speed_;
}

MissionTolerance MissionSection::getTolerance() const
{
  return tolerance_;
}

void MissionSection::setInitialPosition(const MissionPosition& position)
{
  initial_position_ = position;
}

void MissionSection::setFinalPosition(const MissionPosition& position)
{
  final_position_ = position;
}

void MissionSection::setSpeed(const double speed)
{
  speed_ = speed;
}

void MissionSection::setTolerance(const MissionTolerance tolerance)
{
  tolerance_ = tolerance;
}

double MissionSection::x()
{
  return final_position_.getLatitude();
}

double MissionSection::y()
{
  return final_position_.getLongitude();
}

double MissionSection::z()
{
  return final_position_.getZ();
}

