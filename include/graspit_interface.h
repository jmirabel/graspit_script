#ifndef _GRASPIT_INTERFACE_H_
#define _GRASPIT_INTERFACE_H_

// GraspIt! includes
#include <graspit/EGPlanner/searchState.h>
#include <graspit/EGPlanner/simAnnPlanner.h>
#include <graspit/plugin.h>

namespace GraspitInterface {

struct EGPlannerRequest {
  StateType search_space = SPACE_AXIS_ANGLE;
  PlannerType planner = PLANNER_SIM_ANN;
  // List defined in graspit/src/EGPlanner/energy/searchEnergyFactory.cpp
  std::string search_energy = "CONTACT_ENERGY";
  SearchContactType search_contact = CONTACT_PRESET;
  int max_steps = 35000;
  int max_num_results = 50;
};

class GraspitInterface : public QObject,
                         public Plugin {

  Q_OBJECT

private:
  GraspPlanningState *mHandObjectState;
  SimAnnPlanner *mPlanner;

  EGPlannerRequest mRequest;

  bool firstTimeInMainLoop;

  void graspPlanningStateToCout(const GraspPlanningState *gps, Hand *mHand, GraspableBody *mObject);

public:
  GraspitInterface() {}
  ~GraspitInterface() {}

  virtual int init(int argc, char **argv);

  virtual int mainLoop();

// public Q_SLOTS:
public:
  void runPlannerInMainThread();
  void processPlannerResultsInMainThread();
  // void buildFeedbackInMainThread();

// Q_SIGNALS:

  // void emitRunPlannerInMainThread();
  // void emitProcessPlannerResultsInMainThread();
  // void emitBuildFeedbackInMainThread();
};

} // namespace GraspitInterface

#endif
