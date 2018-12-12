//
// Created by narcis on 6/03/18.
//

#include <cola2_control/mission_utils/mission_position.h>

MissionPosition::MissionPosition()
{
}

MissionPosition::~MissionPosition()
{
}

MissionPosition::MissionPosition(const MissionPosition& position):
  latitude_(position.latitude_), longitude_(position.longitude_), z_(position.z_),
  altitude_mode_(position.altitude_mode_)
{
}

MissionPosition::MissionPosition(const double latitude, const double longitude,
                                 const double z, const bool altitude_mode):
  latitude_(latitude), longitude_(longitude), z_(z), altitude_mode_(altitude_mode)
{
}

double MissionPosition::getLatitude() const
{
  return latitude_;
}

double MissionPosition::getLongitude() const
{
  return longitude_;
}

double MissionPosition::getZ() const
{
  return z_;
}

bool MissionPosition::getAltitudeMode() const
{
  return altitude_mode_;
}

void MissionPosition::setLatitude(const double latitude)
{
  latitude_ = latitude;
}

void MissionPosition::setLongitude(const double longitude)
{
  longitude_ = longitude;
}

void MissionPosition::setZ(const double z)
{
  z_ = z;
}

void MissionPosition::setAltitudeMode(const bool altitude_mode)
{
  altitude_mode_ = altitude_mode;
}
