//
// Created by narcis on 6/03/18.
//

#ifndef COLA2_CONTROL_MISSION_POSITION_H
#define COLA2_CONTROL_MISSION_POSITION_H

#include <vector>
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

  MissionPosition(double latitude_, double longitude_, double z_, bool altitude_mode_);

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

  double getAltitudeMode() const;

  void setLatitude(const double latitude);

  void setLongitude(const double longitude);

  void setZ(const double z);

  void setAltitudeMode(const bool altitude_mode);
};

#endif //COLA2_CONTROL_MISSION_POSITION_H
