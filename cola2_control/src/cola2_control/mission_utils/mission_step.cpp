//
// Created by narcis on 6/03/18.
//

#include <cola2_control/mission_utils/mission_step.h>

MissionStep::MissionStep()
{
}

MissionStep::~MissionStep()
{
}

void MissionStep::show()
{
  std::cout << "Maneuver:\n";
  maneuver_->show();
  std::cout << std::endl;
  std::cout << "List of actions:\n";
  for (std::vector<MissionAction>::iterator action = actions_.begin(); action != actions_.end(); ++action)
  {
    action->show();
    std::cout << std::endl;
  }
}

MissionManeuver* MissionStep::getManeuverPtr() const
{
  return maneuver_;
}

std::vector<MissionAction> MissionStep::getActions() const
{
  return actions_;
}

void MissionStep::setManeuverPtr(MissionManeuver* maneuver)
{
  maneuver_ = maneuver;
}

void MissionStep::addAction(MissionAction action)
{
  actions_.push_back(action);
}

unsigned int MissionStep::getStepId()
{
  return step_id_;
}

void MissionStep::incStepId()
{
  step_id_++;
}
