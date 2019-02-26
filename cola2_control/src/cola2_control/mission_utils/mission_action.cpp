//
// Created by narcis on 6/03/18.
//

#include <cola2_control/mission_utils/mission_action.h>

MissionAction::MissionAction()
{
  is_empty_ = true;
}

MissionAction::MissionAction(const std::string& action_id,
                             const std::vector<std::string>& parameters)
    : action_id_(action_id), parameters_(parameters), is_empty_(true)
{
  if (!parameters_.empty())
    is_empty_ = false;
}

MissionAction::~MissionAction()
{
}

std::string MissionAction::getActionId() const
{
  return action_id_;
}

void MissionAction::setActionId(const std::string& value)
{
  action_id_ = value;
}

void MissionAction::addParameters(const std::string& param)
{
  parameters_.push_back(param);
  is_empty_ = false;
}

std::vector<std::string> MissionAction::getParameters() const
{
  return parameters_;
}

bool MissionAction::getIsEmpty() const
{
  return is_empty_;
}
