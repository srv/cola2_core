//
// Created by narcis on 6/03/18.
//

#include <cola2_control/mission_utils/mission.h>

Mission::Mission()
{
}

Mission::~Mission()
{
}

void Mission::loadMission(const std::string& mission_file_name)
{
  // Load XML document file
  TiXmlDocument doc(mission_file_name.c_str());
  if (!doc.LoadFile()) throw std::runtime_error("Invalid/Not found document");

  // Create XML handle
  TiXmlHandle hDoc(&doc);

  // If previous mission, erase it
  mission_.clear();

  // Read all childs of mission tag
  TiXmlElement* pElem = hDoc.FirstChild("mission").FirstChild().Element();
  for (; pElem; pElem = pElem->NextSiblingElement())
  {
    std::string m_name = pElem->Value();
    if (m_name == "mission_step")
    {
      auto step = std::make_shared<MissionStep>();
      loadStep(TiXmlHandle(pElem), *step);
      addStep(step);
    }
  }
}

void Mission::loadStep(TiXmlHandle hDoc, MissionStep& step)
{
  // Search for maneuver
  TiXmlElement* pElem;
  pElem = hDoc.FirstChild("maneuver").Element();
  if (!pElem) throw std::runtime_error("No maneuver element in mission step");

  // Load maneuver according to attribute type
  std::string attribute = pElem->Attribute("type");
  if (attribute == "waypoint")
  {
    auto waypoint = std::make_shared<MissionWaypoint>();
    loadManeuverWaypoint(TiXmlHandle(pElem), *waypoint);
    step.setManeuverPtr(waypoint);
  }
  else if (attribute == "section")
  {
    auto section = std::make_shared<MissionSection>();
    loadManeuverSection(TiXmlHandle(pElem), *section);
    step.setManeuverPtr(section);
  }
  else if (attribute == "park")
  {
    auto park = std::make_shared<MissionPark>();
    loadManeuverPark(TiXmlHandle(pElem), *park);
    step.setManeuverPtr(park);
  }
  else
  {
    throw std::runtime_error("Invalid maneuver type: " + attribute);
  }

  // Load actions
  pElem = hDoc.FirstChild("actions_list").FirstChild().Element();
  for (; pElem; pElem = pElem->NextSiblingElement())
  {
    std::string m_name = pElem->Value();
    if (m_name == "action")
    {
      MissionAction action;
      loadAction(TiXmlHandle(pElem), action);
      step.addAction(action);
    }
  }
}

void Mission::loadManeuverWaypoint(TiXmlHandle hDoc, MissionWaypoint& waypoint)
{
  // Keep track of what has been found
  bool position = false;
  bool speed = false;
  bool tolerance = false;

  // Load tags
  TiXmlElement* pElem = hDoc.FirstChild().Element();
  for (; pElem; pElem = pElem->NextSiblingElement())
  {
    std::string wp_tag = pElem->Value();
    if (wp_tag == "position")
    {
      MissionPosition mission_position;
      loadPosition(TiXmlHandle(pElem), mission_position);
      waypoint.setPosition(mission_position);
      position = true;
    }
    else if (wp_tag == "speed")
    {
      try
      {
        waypoint.setSpeed(std::stod(pElem->GetText()));
        speed = true;
      }
      catch (...) {}  // Nothing to do here, just keep reading
    }
    else if (wp_tag == "tolerance")
    {
      MissionTolerance mission_tolerance;
      loadTolerance(TiXmlHandle(pElem), mission_tolerance);
      waypoint.setTolerance(mission_tolerance);
      tolerance = true;
    }
    if (position && speed && tolerance) break;
  }

  // Check if something is missing
  std::vector<std::string> missing_tags;
  if (!position) missing_tags.push_back("position");
  if (!speed) missing_tags.push_back("speed");
  if (!tolerance) missing_tags.push_back("tolerance");
  if (!missing_tags.empty())
  {
    std::string msg("Waypoint without the following tags:");
    for (const auto& tag : missing_tags) msg += " " + tag;
    throw std::runtime_error(msg);
  }
}

void Mission::loadManeuverSection(TiXmlHandle hDoc, MissionSection& section)
{
  // Keep track of what has been found
  bool initial_position = false;
  bool final_position = false;
  bool speed = false;
  bool tolerance = false;

  // Load tags
  TiXmlElement* pElem = hDoc.FirstChild().Element();
  for (; pElem; pElem = pElem->NextSiblingElement())
  {
    std::string wp_tag = pElem->Value();
    if (wp_tag == "initial_position")
    {
      MissionPosition mission_position;
      loadPosition(TiXmlHandle(pElem), mission_position);
      section.setInitialPosition(mission_position);
      initial_position = true;
    }
    else if (wp_tag == "final_position")
    {
      MissionPosition mission_position;
      loadPosition(TiXmlHandle(pElem), mission_position);
      section.setFinalPosition(mission_position);
      final_position = true;
    }
    else if (wp_tag == "speed")
    {
      try
      {
        section.setSpeed(std::stod(pElem->GetText()));
        speed = true;
      }
      catch (...) {}  // Nothing to do here, just keep reading
    }
    else if (wp_tag == "tolerance")
    {
      MissionTolerance mission_tolerance;
      loadTolerance(TiXmlHandle(pElem), mission_tolerance);
      section.setTolerance(mission_tolerance);
      tolerance = true;
    }
    if (initial_position && final_position && speed && tolerance) break;
  }

  // Check if something is missing
  std::vector<std::string> missing_tags;
  if (!initial_position) missing_tags.push_back("initial_position");
  if (!final_position) missing_tags.push_back("final_position");
  if (!speed) missing_tags.push_back("speed");
  if (!tolerance) missing_tags.push_back("tolerance");
  if (!missing_tags.empty())
  {
    std::string msg("Section without the following tags:");
    for (const auto& tag : missing_tags) msg += " " + tag;
    throw std::runtime_error(msg);
  }
}

void Mission::loadManeuverPark(TiXmlHandle hDoc, MissionPark& park)
{
  // Keep track of what has been found
  bool position = false;
  bool t = false;
  bool tolerance = false;

  // Load tags
  TiXmlElement* pElem = hDoc.FirstChild().Element();
  for (; pElem; pElem = pElem->NextSiblingElement())
  {
    std::string wp_tag = pElem->Value();
    if (wp_tag == "position")
    {
      MissionPosition mission_position;
      loadPosition(TiXmlHandle(pElem), mission_position);
      park.setPosition(mission_position);
      position = true;
    }
    else if (wp_tag == "time")
    {
      try
      {
        park.setTime(std::stod(pElem->GetText()));
        t = true;
      }
      catch (...) {}  // Nothing to do here, just keep reading
    }
    else if (wp_tag == "tolerance")
    {
      MissionTolerance mission_tolerance;
      loadTolerance(TiXmlHandle(pElem), mission_tolerance);
      park.setTolerance(mission_tolerance);
      tolerance = true;
    }
    if (position && t && tolerance) break;
  }

  // Check if something is missing
  std::vector<std::string> missing_tags;
  if (!position) missing_tags.push_back("position");
  if (!t) missing_tags.push_back("time");
  if (!tolerance) missing_tags.push_back("tolerance");
  if (!missing_tags.empty())
  {
    std::string msg("Park without the following tags:");
    for (const auto& tag : missing_tags) msg += " " + tag;
    throw std::runtime_error(msg);
  }
}

void Mission::loadAction(TiXmlHandle hDoc, MissionAction& action)
{
  // Find action in action_list
  TiXmlElement* pElem = hDoc.FirstChild().Element();
  if (!pElem) throw std::runtime_error("No action in action_list");

  // Find action_id
  std::string action_tag = pElem->Value();
  if (action_tag != "action_id") throw std::runtime_error("Expected action_id");
  action.setActionId(pElem->GetText());

  // Check params
  pElem = pElem->NextSiblingElement();
  if (!pElem || (std::string(pElem->Value()) == "parameters"))  // Was (!pElem || pElem->Value() == "parameters")
    return;  // No parameters
  hDoc = TiXmlHandle(pElem);

  // TODO: Check that parameters are really stored!
  pElem = hDoc.FirstChild().Element();
  for (; pElem; pElem = pElem->NextSiblingElement())
  {
    std::string param_tag = pElem->Value();
    std::string param = pElem->GetText();
    if (param_tag == "param")
    {
      action.addParameters(param);
    }
  }
}

void Mission::loadPosition(TiXmlHandle hDoc, MissionPosition& position)
{
  // Keep track of what has been found
  bool lat = false;
  bool lon = false;
  bool z = false;
  bool mode = false;

  // Load tags
  TiXmlElement* pElem = hDoc.FirstChild().Element();
  for (; pElem; pElem = pElem->NextSiblingElement())
  {
    std::string wp_tag = pElem->Value();
    if (wp_tag == "latitude")
    {
      try
      {
        position.setLatitude(std::stod(pElem->GetText()));
        lat = true;
      }
      catch (...) {}  // Nothing to do here, just keep reading
    }
    else if (wp_tag == "longitude")
    {
      try
      {
        position.setLongitude(std::stod(pElem->GetText()));
        lon = true;
      }
      catch (...) {}  // Nothing to do here, just keep reading
    }
    else if (wp_tag == "z")
    {
      try
      {
        position.setZ(std::stod(pElem->GetText()));
        z = true;
      }
      catch (...) {}  // Nothing to do here, just keep reading
    }
    else if (wp_tag == "altitude_mode")
    {
      std::string mode = pElem->GetText();
      std::transform(mode.begin(), mode.end(), mode.begin(), ::tolower);
      if (mode == "true") position.setAltitudeMode(true);
      else position.setAltitudeMode(false);
      mode = true;
    }
    if (lat && lon && z && mode) break;
  }

  // If no altitude_mode but z found, assume altitude_mode is false
  if (lat && lon && z && (!mode))
  {
    position.setAltitudeMode(false);
    mode = true;
  }

  // Check if something is missing
  std::vector<std::string> missing_tags;
  if (!lat) missing_tags.push_back("latitude");
  if (!lon) missing_tags.push_back("longitude");
  if (!z) missing_tags.push_back("z");
  if (!mode) missing_tags.push_back("altitude_mode");
  if (!missing_tags.empty())
  {
    std::string msg("Position without the following tags:");
    for (const auto& tag : missing_tags) msg += " " + tag;
    throw std::runtime_error(msg);
  }
}

void Mission::loadTolerance(TiXmlHandle hDoc, MissionTolerance& tolerance)
{
  // Keep track of what has been found
  bool x = false;
  bool y = false;
  bool z = false;

  // Load tags
  TiXmlElement* pElem = hDoc.FirstChild().Element();
  for (; pElem; pElem = pElem->NextSiblingElement())
  {
    std::string wp_tag = pElem->Value();
    if (wp_tag == "x")
    {
      try
      {
        tolerance.setX(std::stod(pElem->GetText()));
        x = true;
      }
      catch (...) {}  // Nothing to do here, just keep reading
    }
    else if (wp_tag == "y")
    {
      try
      {
        tolerance.setY(std::stod(pElem->GetText()));
        y = true;
      }
      catch (...) {}  // Nothing to do here, just keep reading
    }
    else if (wp_tag == "z")
    {
      try
      {
        tolerance.setZ(std::stod(pElem->GetText()));
        z = true;
      }
      catch (...) {}  // Nothing to do here, just keep reading
    }
    if (x && y && z) break;
  }

  // Check if something is missing
  std::vector<std::string> missing_tags;
  if (!x) missing_tags.push_back("x");
  if (!y) missing_tags.push_back("y");
  if (!z) missing_tags.push_back("z");
  if (!missing_tags.empty())
  {
    std::string msg("Tolerance without the following tags:");
    for (const auto& tag : missing_tags) msg += " " + tag;
    throw std::runtime_error(msg);
  }
}

void Mission::writeMission(std::string mission_file_name)
{
  TiXmlDocument doc;
  TiXmlDeclaration* decl = new TiXmlDeclaration("1.0", "", "");
  doc.LinkEndChild(decl);
  TiXmlElement* mission = new TiXmlElement("mission");
  doc.LinkEndChild(mission);
  for (const auto& step_ptr : mission_) writeMissionStep(mission, *step_ptr);
  doc.SaveFile(mission_file_name.c_str());
}

void Mission::writeMissionStep(TiXmlElement* mission, const MissionStep& step)
{
  TiXmlElement* mission_step = new TiXmlElement("mission_step");

  // Write mission step maneuver
  if (step.getManeuverPtr()->getManeuverType() == WAYPOINT_MANEUVER)
  {
    auto wp = std::dynamic_pointer_cast<MissionWaypoint>(step.getManeuverPtr());
    writeManeuverWaypoint(mission_step, *wp);
  }
  else if (step.getManeuverPtr()->getManeuverType() == SECTION_MANEUVER)
  {
    auto sec = std::dynamic_pointer_cast<MissionSection>(step.getManeuverPtr());
    writeManeuverSection(mission_step, *sec);
  }
  else if (step.getManeuverPtr()->getManeuverType() == PARK_MANEUVER)
  {
    auto park = std::dynamic_pointer_cast<MissionPark>(step.getManeuverPtr());
    writeManeuverPark(mission_step, *park);
  }

  // Write action_list if available
  std::vector<MissionAction> actions = step.getActions();
  if (!actions.empty())
  {
    TiXmlElement* action_list = new TiXmlElement("actions_list");
    for (const auto& action : actions) writeAction(action_list, action);
    mission_step->LinkEndChild(action_list);
  }
  mission->LinkEndChild(mission_step);
}

void Mission::writeManeuverWaypoint(TiXmlElement* mission, const MissionWaypoint& wp)
{
  TiXmlElement* element = new TiXmlElement("maneuver");
  element->SetAttribute("type", "waypoint");
  mission->LinkEndChild(element);
  writeManeuverPosition(element, wp.getPosition(), "position");
  writeManeuverTolerance(element, wp.getTolerance());
  TiXmlElement* speed = new TiXmlElement("speed");
  speed->LinkEndChild(new TiXmlText(std::to_string(wp.getSpeed())));
  element->LinkEndChild(speed);
}

void Mission::writeManeuverSection(TiXmlElement* mission, const MissionSection& sec)
{
  TiXmlElement* element = new TiXmlElement("maneuver");
  element->SetAttribute("type", "section");
  mission->LinkEndChild(element);
  writeManeuverPosition(element, sec.getInitialPosition(), "initial_position");
  writeManeuverPosition(element, sec.getFinalPosition(), "final_position");
  writeManeuverTolerance(element, sec.getTolerance());
  TiXmlElement* speed = new TiXmlElement("speed");
  speed->LinkEndChild(new TiXmlText(std::to_string(sec.getSpeed())));
  element->LinkEndChild(speed);
}

void Mission::writeManeuverPark(TiXmlElement* mission, const MissionPark& park)
{
  TiXmlElement* element = new TiXmlElement("maneuver");
  element->SetAttribute("type", "park");
  mission->LinkEndChild(element);
  writeManeuverPosition(element, park.getPosition(), "position");
  writeManeuverTolerance(element, park.getTolerance());
  TiXmlElement* time = new TiXmlElement("time");
  time->LinkEndChild(new TiXmlText(std::to_string(park.getTime())));
  element->LinkEndChild(time);
}

void Mission::writeAction(TiXmlElement* mission, const MissionAction& action)
{
  TiXmlElement* element = new TiXmlElement("action");
  TiXmlElement* action_id = new TiXmlElement("action_id");
  action_id->LinkEndChild(new TiXmlText(action.getActionId()));
  element->LinkEndChild(action_id);
  if (action.getParameters().size() > 0)
  {
    TiXmlElement* parameters = new TiXmlElement("parameters");
    element->LinkEndChild(parameters);
    for (const auto& elem : action.getParameters())
    {
      TiXmlElement* param = new TiXmlElement("param");
      param->LinkEndChild(new TiXmlText(elem));
      parameters->LinkEndChild(param);
    }
  }
  mission->LinkEndChild(element);
}

void Mission::writeManeuverPosition(TiXmlElement* maneuver, const MissionPosition& p, std::string position_tag)
{
  TiXmlElement* position = new TiXmlElement(position_tag);
  maneuver->LinkEndChild(position);
  TiXmlElement* lat = new TiXmlElement("latitude");
  lat->LinkEndChild(new TiXmlText(std::to_string(p.getLatitude())));
  position->LinkEndChild(lat);
  TiXmlElement* lon = new TiXmlElement("longitude");
  lon->LinkEndChild(new TiXmlText(std::to_string(p.getLongitude())));
  position->LinkEndChild(lon);
  TiXmlElement* z = new TiXmlElement("z");
  z->LinkEndChild(new TiXmlText(std::to_string(p.getZ())));
  position->LinkEndChild(z);
  TiXmlElement* mode = new TiXmlElement("altitude_mode");
  if (p.getAltitudeMode())
    mode->LinkEndChild(new TiXmlText("true"));
  else
    mode->LinkEndChild(new TiXmlText("false"));
  position->LinkEndChild(mode);
}

void Mission::writeManeuverTolerance(TiXmlElement* maneuver, const MissionTolerance& tol)
{
  TiXmlElement* tolerance = new TiXmlElement("tolerance");
  maneuver->LinkEndChild(tolerance);
  TiXmlElement* x = new TiXmlElement("x");
  x->LinkEndChild(new TiXmlText(std::to_string(tol.getX())));
  tolerance->LinkEndChild(x);
  TiXmlElement* y = new TiXmlElement("y");
  y->LinkEndChild(new TiXmlText(std::to_string(tol.getY())));
  tolerance->LinkEndChild(y);
  TiXmlElement* z = new TiXmlElement("z");
  z->LinkEndChild(new TiXmlText(std::to_string(tol.getZ())));
  tolerance->LinkEndChild(z);
}

void Mission::addStep(std::shared_ptr<MissionStep> step)
{
  mission_.push_back(step);
}

std::shared_ptr<MissionStep> Mission::getStep(std::size_t i)
{
  assert(i < mission_.size());
  return mission_[i];
}

std::size_t Mission::size()
{
  return mission_.size();
}
