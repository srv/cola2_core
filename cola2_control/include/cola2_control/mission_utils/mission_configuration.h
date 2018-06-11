//
// Created by narcis on 6/03/18.
//

#ifndef COLA2_CONTROL_MISSION_CONFIGURATION_H
#define COLA2_CONTROL_MISSION_CONFIGURATION_H

#include <vector>
#include <string>
#include <iostream>


/**
 * \brief MissionConfiguration class from mission types.
 */
class MissionConfiguration
{
private:
  std::string key_;
  std::string value_;

public:
  MissionConfiguration();

  ~MissionConfiguration();

  MissionConfiguration(std::string key, std::string value);

  friend std::ostream& operator<<(std::ostream& stream, const MissionConfiguration& conf)
  {
    stream << "configuration -> " << conf.getKey() << ": " << conf.getValue();
    return stream;
  }

  std::string  getKey() const;

  std::string  getValue() const;
};

#endif //COLA2_CONTROL_MISSION_CONFIGURATION_H
