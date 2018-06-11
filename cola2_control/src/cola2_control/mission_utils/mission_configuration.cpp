//
// Created by narcis on 6/03/18.
//

#include <cola2_control/mission_utils/mission_configuration.h>

MissionConfiguration::MissionConfiguration()
{
}

MissionConfiguration::MissionConfiguration(std::string key, std::string value) : key_(key), value_(value)
{
}

MissionConfiguration::~MissionConfiguration()
{
}

std::string  MissionConfiguration::getKey() const
{
  return key_;
}

std::string  MissionConfiguration::getValue() const
{
  return value_;
}