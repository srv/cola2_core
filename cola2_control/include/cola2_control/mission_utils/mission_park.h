//
// Created by narcis on 6/03/18.
//

#ifndef COLA2_CONTROL_MISSION_PARK_H
#define COLA2_CONTROL_MISSION_PARK_H

#include <vector>
#include <string>
#include <iostream>
#include <cola2_control/mission_utils/mission_position.h>
#include <cola2_control/mission_utils/mission_tolerance.h>
#include <cola2_control/mission_utils/mission_maneuver.h>


/**
 * \brief MissionPark class from mission types.
 */
class MissionPark : public MissionManeuver
{
private:
  MissionPosition position_;
  unsigned int time_;
  MissionTolerance tolerance_;

public:
  MissionPark();

  ~MissionPark();

  MissionPark(MissionPosition position_, unsigned int time_, MissionTolerance tolerance_);

//  friend std::ostream& operator<<(std::ostream& stream, const MissionPark& p)
//  {
//    stream << "Park -> " << p.getPosition() << " for " << p.getTime() << "s";
//  }

  MissionPosition getPosition() const;

  unsigned int getTime() const;

  MissionTolerance getTolerance() const;

  void setPosition(const MissionPosition position);

  void setTime(const unsigned int time);

  void setTolerance(const MissionTolerance tolerance);

  double x();

  double y();

  double z();
};

#endif //COLA2_CONTROL_MISSION_PARK_H
