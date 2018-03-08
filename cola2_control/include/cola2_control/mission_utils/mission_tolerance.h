//
// Created by narcis on 6/03/18.
//

#ifndef COLA2_CONTROL_MISSION_TOLERANCE_H
#define COLA2_CONTROL_MISSION_TOLERANCE_H

#include <vector>
#include <string>
#include <iostream>

/**
 * \brief MissionTolerance class from mission types.
 */
class MissionTolerance
{
private:
  double x_;
  double y_;
  double z_;

public:
  MissionTolerance();

  ~MissionTolerance();

  MissionTolerance(const MissionTolerance& tolerance);

  MissionTolerance(double x_, double y_, double z_);

  friend std::ostream& operator<<(std::ostream& stream, const MissionTolerance& tol)
  {
    stream << "[" << tol.getX() << ", " << tol.getY() << ", " << tol.getZ() << "]";
  }

  double getX() const;

  double getY() const;

  double getZ() const;

  void setX(const double x);

  void setY(const double y);

  void setZ(const double z);
};

#endif //COLA2_CONTROL_MISSION_TOLERANCE_H
