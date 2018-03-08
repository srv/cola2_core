//
// Created by narcis on 6/03/18.
//

#include <cola2_control/mission_utils/mission.h>

Mission::Mission() : step_size_(0)
{
}

Mission::~Mission()
{
}

MissionStep* Mission::getStep(unsigned int i)
{
  assert(i < mission_.size());
  return mission_.at(i);
}

unsigned int Mission::size()
{
  return mission_.size();
}

std::string Mission::to_string(const double value) const
{
  std::ostringstream sstream;
  sstream << value;
  return sstream.str();
}

void Mission::addStep(MissionStep* step)
{
  step->incStepId();
  mission_.push_back(step);
}

int Mission::loadAction(TiXmlHandle hDoc, MissionAction& action)
{
  TiXmlElement* pElem;
  pElem = hDoc.FirstChild().Element();
  if (!pElem)
  {
    std::cerr << "No action element.\n";
    return -5;  // No action element
  }
  std::string action_tag = pElem->Value();
  if (action_tag != "action_id")
  {
    std::cerr << "Error: action_id expected.\n";
    return -6;  // Invalid action_id tag
  }
  action.setActionId(pElem->GetText());

  // Check params
  pElem = pElem = pElem->NextSiblingElement();
  if (!pElem || pElem->Value() == "parameters")
    return 0;  // No parameters
  hDoc = TiXmlHandle(pElem);

  // TODO: Check that parameters are really stored!
  pElem = hDoc.FirstChild().Element();
  for (pElem; pElem; pElem = pElem->NextSiblingElement())
  {
    std::string param_tag = pElem->Value();
    std::string param = pElem->GetText();
    if (param_tag == "param")
    {
      action.addParameters(param);
    }
  }
}

bool Mission::loadPosition(TiXmlHandle hDoc, MissionPosition& position)
{
  TiXmlElement* pElem;
  pElem = hDoc.FirstChild().Element();
  bool lat = false;
  bool lon = false;
  bool z_ = false;
  bool mode = false;

  for (pElem; pElem; pElem = pElem->NextSiblingElement())
  {
    std::string wp_tag = pElem->Value();
    if (wp_tag == "latitude")
    {
      position.setLatitude(atof(pElem->GetText()));
      lat = true;
    }
    else if (wp_tag == "longitude")
    {
      position.setLongitude(atof(pElem->GetText()));
      lon = true;
    }
    else if (wp_tag == "z")
    {
      position.setZ(atof(pElem->GetText()));
      z_ = true;
    }
    else if (wp_tag == "altitude_mode")
    {
      std::string mode = pElem->GetText();
      std::cout << "ALTITUDE MODE: " << mode << "\n";
      if (mode == "true" || mode == "True")
      {
        position.setAltitudeMode(true);
      }
      else
      {
        position.setAltitudeMode(false);
      }
      mode = true;
    }
  }
//  std::cout << "Position:\n ";
//  std::cout << position << std::endl;
  return (lat && lon && z_ && mode);
}

bool Mission::loadTolerance(TiXmlHandle hDoc, MissionTolerance& tolerance)
{
  TiXmlElement* pElem;
  pElem = hDoc.FirstChild().Element();
  bool x_ = false;
  bool y_ = false;
  bool z_ = false;

  for (pElem; pElem; pElem = pElem->NextSiblingElement())
  {
    std::string wp_tag = pElem->Value();
    if (wp_tag == "x")
    {
      tolerance.setX(atof(pElem->GetText()));
      x_ = true;
    }
    else if (wp_tag == "y")
    {
      tolerance.setY(atof(pElem->GetText()));
      y_ = true;
    }
    else if (wp_tag == "z")
    {
      tolerance.setZ(atof(pElem->GetText()));
      z_ = true;
    }
    if (x_ && y_ && z_)
      break;
  }
//  std::cout << "Tolerance :\n";
//  std::cout << tolerance << std::endl;
  return (x_ && y_ && z_);
}

bool Mission::loadManeuverWaypoint(TiXmlHandle hDoc, MissionWaypoint& waypoint)
{
  TiXmlElement* pElem;
  pElem = hDoc.FirstChild().Element();
  bool position = false;
  bool speed = false;
  bool tolerance = false;
  std::cout << "Load maneuver waypoint\n";
  for (pElem; pElem; pElem = pElem->NextSiblingElement())
  {
    std::string wp_tag = pElem->Value();
    if (wp_tag == "position")
    {
      MissionPosition tmp;
      position = loadPosition(TiXmlHandle(pElem), tmp);
      waypoint.setPosition(tmp);
    }
    else if (wp_tag == "speed")
    {
      waypoint.setSpeed(atof(pElem->GetText()));
      speed = true;
    }
    else if (wp_tag == "tolerance")
    {
      MissionTolerance tmp;
      tolerance = loadTolerance(TiXmlHandle(pElem), tmp);
      waypoint.setTolerance(tmp);
    }
    if (position && speed && tolerance)
      break;
  }
  std::cout << "Loaded!" << std::endl;
  return (position && speed && tolerance);
}

bool Mission::loadManeuverSection(TiXmlHandle hDoc, MissionSection& section)
{
  TiXmlElement* pElem;
  pElem = hDoc.FirstChild().Element();
  bool initial_position = false;
  bool final_position = false;
  bool speed = false;
  bool tolerance = false;

  for (pElem; pElem; pElem = pElem->NextSiblingElement())
  {
    std::string wp_tag = pElem->Value();
    if (wp_tag == "initial_position")
    {
      MissionPosition tmp;
      initial_position = loadPosition(TiXmlHandle(pElem), tmp);
      section.setInitialPosition(tmp);
      // std::cout << "initial_position.mode: " << section.initial_position.altitude_mode << "\n";
    }
    else if (wp_tag == "final_position")
    {
      MissionPosition tmp;
      final_position = loadPosition(TiXmlHandle(pElem), tmp);
      section.setFinalPosition(tmp);
      // std::cout << "final_position.mode: " << section.initial_position.altitude_mode << "\n";
    }
    else if (wp_tag == "speed")
    {
      section.setSpeed(atof(pElem->GetText()));
      speed = true;
    }
    else if (wp_tag == "tolerance")
    {
      MissionTolerance tmp;
      tolerance = loadTolerance(TiXmlHandle(pElem), tmp);
      section.setTolerance(tmp);
    }
    if (initial_position && final_position && speed && tolerance)
      break;
  }
  // std::cout << "Loadede " << section << std::endl;
  return (initial_position && final_position && speed && tolerance);
}

bool Mission::loadManeuverPark(TiXmlHandle hDoc, MissionPark& park)
{
  TiXmlElement* pElem;
  pElem = hDoc.FirstChild().Element();
  bool position = false;
  bool time = false;
  bool tolerance = false;

  for (pElem; pElem; pElem = pElem->NextSiblingElement())
  {
    std::string wp_tag = pElem->Value();
    if (wp_tag == "position")
    {
      MissionPosition tmp;
      position = loadPosition(TiXmlHandle(pElem), tmp);
      park.setPosition(tmp);
    }
    else if (wp_tag == "time")
    {
      park.setTime(atof(pElem->GetText()));
      time = true;
    }
    else if (wp_tag == "tolerance")
    {
      MissionTolerance tmp;
      tolerance = loadTolerance(TiXmlHandle(pElem), tmp);
      park.setTolerance(tmp);
    }
    if (position && time && tolerance)
      break;
  }
  // std::cout << "Load: " << park << std::endl;
  return (position && time && tolerance);
}

int Mission::loadStep(TiXmlHandle hDoc, MissionStep& step)
{
  TiXmlElement* pElem;
  pElem = hDoc.FirstChild("maneuver").Element();
  if (!pElem)
  {
    std::cout << "No maneuver element in mission step!\n";
    return -3;  // No key element
  }
  else
  {
    std::string attribute = pElem->Attribute("type");
    if (attribute == "waypoint")
    {
      std::cout << "Waypoint maneuver found " << std::endl;
      MissionWaypoint* waypoint = new MissionWaypoint();
      loadManeuverWaypoint(TiXmlHandle(pElem), *waypoint);
      step.setManeuverPtr(waypoint);
    }
    else if (attribute == "section")
    {
      std::cout << "Section maneuver found " << std::endl;
      MissionSection* section = new MissionSection();
      loadManeuverSection(TiXmlHandle(pElem), *section);
      step.setManeuverPtr(section);
    }
    else if (attribute == "park")
    {
      MissionPark* park = new MissionPark();
      loadManeuverPark(TiXmlHandle(pElem), *park);
      step.setManeuverPtr(park);
    }
    else
    {
      std::cout << "Invalid maneuver type: " << attribute << std::endl;
    }
  }

  pElem = hDoc.FirstChild("actions_list").FirstChild().Element();
  if (!pElem)
  {
    std::cout << "No actions found in mission step!\n";
  }
  else
  {
    std::cout << "Actions found in mission step!\n";
    for (pElem; pElem; pElem = pElem->NextSiblingElement())
    {
      std::string m_name = pElem->Value();
      std::cout << "found: " << m_name << std::endl;
      if (m_name == "action")
      {
        std::cout << "Load action ...\n";
        MissionAction action;
        loadAction(TiXmlHandle(pElem), action);
        step.addAction(action);
      }
    }
  }
  return 0;  // Everything ok
}

int Mission::loadMission(const std::string mission_file_name)
{
  std::cout << "Load mission " << mission_file_name << std::endl;

  // Load XML document
  TiXmlDocument doc(mission_file_name.c_str());
  if (!doc.LoadFile())
  {
    std::cout << "Invalid/Not found document.\n";
    return -1;  // Invalid/Not found document
  }

  TiXmlHandle hDoc(&doc);
  TiXmlElement* pElem;

  // If previous mission erase it
  mission_.clear();

  // Read all childs of mission tag
  pElem = hDoc.FirstChild("mission").FirstChild().Element();
  std::string m_name = pElem->Value();
  for (pElem; pElem; pElem = pElem->NextSiblingElement())
  {
    std::string m_name = pElem->Value();
    if (m_name == "mission_step")
    {
      std::cout << "Mission step found " << std::endl;
      MissionStep* step = new MissionStep();
      loadStep(TiXmlHandle(pElem), *step);
      addStep(step);
    }
    else
    {
      std::cout << "Error reading mission step. Found " << m_name << ".\n";
    }
  }
  return 0;
}

int Mission::writeAction(TiXmlElement* mission, MissionAction& action)
{
  std::cout << "Write: " << action.getActionId() << std::endl;

  TiXmlElement* element = new TiXmlElement("action");
  TiXmlElement* action_id = new TiXmlElement("action_id");
  action_id->LinkEndChild(new TiXmlText(action.getActionId()));
  element->LinkEndChild(action_id);
  if (action.getParameters().size() > 0)
  {
    TiXmlElement* parameters = new TiXmlElement("parameters");
    element->LinkEndChild(parameters);
    for (std::vector<std::string>::iterator i = action.getParameters().begin(); i != action.getParameters().end(); i++)
    {
      TiXmlElement* param = new TiXmlElement("param");
      param->LinkEndChild(new TiXmlText(*i));
      parameters->LinkEndChild(param);
    }
  }
  mission->LinkEndChild(element);
  return 0;
}

int Mission::writeManeuverPosition(TiXmlElement* maneuver, const MissionPosition& p, std::string position_tag)
{
  TiXmlElement* position = new TiXmlElement(position_tag);
  maneuver->LinkEndChild(position);
  TiXmlElement* lat = new TiXmlElement("latitude");
  lat->LinkEndChild(new TiXmlText(to_string(p.getLatitude())));
  position->LinkEndChild(lat);
  TiXmlElement* lon = new TiXmlElement("longitude");
  lon->LinkEndChild(new TiXmlText(to_string(p.getLongitude())));
  position->LinkEndChild(lon);
  TiXmlElement* z = new TiXmlElement("z");
  z->LinkEndChild(new TiXmlText(to_string(p.getZ())));
  position->LinkEndChild(z);
  TiXmlElement* mode = new TiXmlElement("altitude_mode");
  if (p.getAltitudeMode())
    mode->LinkEndChild(new TiXmlText("true"));
  else
    mode->LinkEndChild(new TiXmlText("false"));
  position->LinkEndChild(mode);
  return 0;
}

int Mission::writeManeuverTolerance(TiXmlElement* maneuver, const MissionTolerance& tol)
{
  TiXmlElement* tolerance = new TiXmlElement("tolerance");
  maneuver->LinkEndChild(tolerance);
  TiXmlElement* x = new TiXmlElement("x");
  x->LinkEndChild(new TiXmlText(to_string(tol.getX())));
  tolerance->LinkEndChild(x);
  TiXmlElement* y = new TiXmlElement("y");
  y->LinkEndChild(new TiXmlText(to_string(tol.getY())));
  tolerance->LinkEndChild(y);
  TiXmlElement* z = new TiXmlElement("z");
  z->LinkEndChild(new TiXmlText(to_string(tol.getZ())));
  tolerance->LinkEndChild(z);
  return 0;
}

int Mission::writeManeuverWaypoint(TiXmlElement* mission, const MissionWaypoint& wp)
{
//  std::cout << "Write: " << wp << std::endl;
  TiXmlElement* element = new TiXmlElement("maneuver");
  element->SetAttribute("type", "waypoint");
  mission->LinkEndChild(element);
  writeManeuverPosition(element, wp.getPosition(), "position");
  writeManeuverTolerance(element, wp.getTolerance());
  TiXmlElement* speed = new TiXmlElement("speed");
  speed->LinkEndChild(new TiXmlText(to_string(wp.getSpeed())));
  element->LinkEndChild(speed);
  return 0;
}

int Mission::writeManeuverSection(TiXmlElement* mission, const MissionSection& sec)
{
//  std::cout << "Write: " << sec << std::endl;
  TiXmlElement* element = new TiXmlElement("maneuver");
  element->SetAttribute("type", "section");
  mission->LinkEndChild(element);
  writeManeuverPosition(element, sec.getInitialPosition(), "initial_position");
  writeManeuverPosition(element, sec.getFinalPosition(), "final_position");
  writeManeuverTolerance(element, sec.getTolerance());
  TiXmlElement* speed = new TiXmlElement("speed");
  speed->LinkEndChild(new TiXmlText(to_string(sec.getSpeed())));
  element->LinkEndChild(speed);
  return 0;
}

int Mission::writeManeuverPark(TiXmlElement* mission, const MissionPark& park)
{
//  std::cout << "Write: " << park << std::endl;
  TiXmlElement* element = new TiXmlElement("maneuver");
  element->SetAttribute("type", "park");
  mission->LinkEndChild(element);
  writeManeuverPosition(element, park.getPosition(), "position");
  writeManeuverTolerance(element, park.getTolerance());
  TiXmlElement* time = new TiXmlElement("time");
  time->LinkEndChild(new TiXmlText(to_string(park.getTime())));
  element->LinkEndChild(time);
  return 0;
}

int Mission::writeMissionStep(TiXmlElement* mission, const MissionStep& step)
{
//  std::cout << "Write: " << step << std::endl;
  TiXmlElement* mission_step = new TiXmlElement("mission_step");

  // Write mission step maneuver
  if (step.getManeuverPtr()->getManeuverType() == WAYPOINT_MANEUVER)
  {
    std::cout << "Add waypoint\n";
    MissionWaypoint* wp = static_cast<MissionWaypoint*>(step.getManeuverPtr());
    writeManeuverWaypoint(mission_step, *wp);
  }
  else if (step.getManeuverPtr()->getManeuverType() == SECTION_MANEUVER)
  {
    std::cout << "Add section\n";
    MissionSection* sec = static_cast<MissionSection*>(step.getManeuverPtr());
    writeManeuverSection(mission_step, *sec);
  }
  else if (step.getManeuverPtr()->getManeuverType() == PARK_MANEUVER)
  {
    std::cout << "Add park\n";
    MissionPark* park = static_cast<MissionPark*>(step.getManeuverPtr());
    writeManeuverPark(mission_step, *park);
  }

  // Write action_list if available
  std::vector<MissionAction> actions = step.getActions();
  if (actions.size() > 0)
  {
    TiXmlElement* action_list = new TiXmlElement("actions_list");
    for (std::vector<MissionAction>::iterator action = actions.begin(); action != actions.end(); ++action)
    {
      writeAction(action_list, *action);
    }
    mission_step->LinkEndChild(action_list);
  }
  mission->LinkEndChild(mission_step);

  return 0;
}

int Mission::writeMission(std::string mission_file_name)
{
  TiXmlDocument doc;
  TiXmlDeclaration* decl = new TiXmlDeclaration("1.0", "", "");
  doc.LinkEndChild(decl);
  TiXmlElement* mission = new TiXmlElement("mission");
  doc.LinkEndChild(mission);

  for (unsigned int i = 0; i < mission_.size(); i++)
  {
    MissionStep* step = mission_.at(i);
    writeMissionStep(mission, *step);
  }
  doc.SaveFile(mission_file_name.c_str());
  return 0;
}
