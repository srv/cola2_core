//
// Created by narcis on 6/03/18.
//

#include <cola2_control/mission_utils/mission_tolerance.h>

MissionTolerance::MissionTolerance()
{
}

MissionTolerance::~MissionTolerance()
{
}

MissionTolerance::MissionTolerance(const MissionTolerance& tolerance):
  x_(tolerance.x_), y_(tolerance.y_), z_(tolerance.z_)
{
}

MissionTolerance::MissionTolerance(const double x, const double y, const double z): x_(x), y_(y), z_(z)
{
}

double MissionTolerance::getX() const
{
  return x_;
}

double MissionTolerance::getY() const
{
  return y_;
}

double MissionTolerance::getZ() const
{
  return z_;
}

void MissionTolerance::setX(const double x)
{
  x_ = x;
}

void MissionTolerance::setY(const double y)
{
  y_ = y;
}

void MissionTolerance::setZ(const double z)
{
  z_ = z;
}
