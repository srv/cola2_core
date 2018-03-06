//
// Created by narcis on 6/03/18.
//

#ifndef COLA2_CONTROL_MISSION_ACTION_H
#define COLA2_CONTROL_MISSION_ACTION_H

#include <vector>
#include <string>
#include <iostream>

/**
 * \brief MissionAction class from mission types.
 */
class MissionAction
{
private:
  std::string action_id_;
  std::vector<std::string> parameters_;
  bool is_empty_;

public:
  MissionAction();

  MissionAction(std::string action_id_, std::vector<std::string> parameters_);

  ~MissionAction();

  friend std::ostream& operator<<(std::ostream& stream, const MissionAction& a)
  {
    stream << "Action " << a.getActionId();
    if (!a.getIsEmpty())
    {
      stream << ": with " << a.getParameters().size() << " params: ";
      for (std::vector<std::string>::const_iterator i = a.getParameters().begin(); i != a.getParameters().end(); i++)
      {
        stream << *i << ", ";
      }
    }
  }

  void setActionId(const std::string value);
  std::string getActionId() const;
  void addParameters(const std::string param);
  std::vector<std::string> getParameters() const;
  void show();
  bool getIsEmpty() const;
};
#endif //COLA2_CONTROL_MISSION_ACTION_H
