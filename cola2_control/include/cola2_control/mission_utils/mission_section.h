//
// Created by narcis on 6/03/18.
//

#ifndef COLA2_CONTROL_MISSION_SECTION_H
#define COLA2_CONTROL_MISSION_SECTION_H

#include <vector>
#include <string>
#include <iostream>
#include <cola2_control/mission_utils/mission_position.h>
#include <cola2_control/mission_utils/mission_tolerance.h>
#include <cola2_control/mission_utils/mission_maneuver.h>

/**
 * \brief MissionSection class from mission types.
 */
class MissionSection : public MissionManeuver
{
private:
  MissionPosition initial_position_;
  MissionPosition final_position_;
  double speed_;
  MissionTolerance tolerance_;

public:
  MissionSection();

  ~MissionSection();

  MissionSection(MissionPosition initial_position_, MissionPosition final_position_, double speed_,
                 MissionTolerance tolerance_);

//  friend std::ostream& operator<<(std::ostream& stream, const MissionSection& s)
//  {
//    stream << "Section -> " << s.getInitialPosition() << " to " << s.getFinalPosition();
//  }

  MissionPosition getInitialPosition() const;

  MissionPosition getFinalPosition() const;

  double getSpeed() const;

  MissionTolerance getTolerance() const;

  void setInitialPosition(const MissionPosition position);

  void setFinalPosition(const MissionPosition position);

  void setSpeed(const double speed);

  void setTolerance(const MissionTolerance tolerance);

  double x();

  double y();

  double z();
};

#endif //COLA2_CONTROL_MISSION_SECTION_H
