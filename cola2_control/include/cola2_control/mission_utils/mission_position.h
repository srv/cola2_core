//
// Created by narcis on 6/03/18.
//

#ifndef COLA2_CONTROL_MISSION_POSITION_H
#define COLA2_CONTROL_MISSION_POSITION_H

#include <string>
#include <iostream>

/**
 * \brief MissionPosition class from mission types.
 */
class MissionPosition
{
private:
  double latitude_;
  double longitude_;
  double z_;
  bool altitude_mode_;

public:
  MissionPosition();

  ~MissionPosition();

  MissionPosition(const MissionPosition& position);

  MissionPosition(const double latitude, const double longitude, const double z, const bool altitude_mode);

  friend std::ostream& operator<<(std::ostream& stream, const MissionPosition& pos)
  {
    stream << "[" << pos.getLatitude() << ", " << pos.getLongitude() << ", " << pos.getZ();
    if (pos.getAltitudeMode())
      stream << " (altitude)]";
    else
      stream << " (depth)]";
    return stream;
  }

  double getLatitude() const;

  double getLongitude() const;

  double getZ() const;

  bool getAltitudeMode() const;

  void setLatitude(const double latitude);

  void setLongitude(const double longitude);

  void setZ(const double z);

  void setAltitudeMode(const bool altitude_mode);
};

#endif //COLA2_CONTROL_MISSION_POSITION_H
