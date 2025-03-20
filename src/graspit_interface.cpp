#include "graspit_interface.h"

#include <graspit/graspitCore.h>
#include <graspit/ivmgr.h>
#include <graspit/robot.h>
#include <graspit/world.h>
#include <string>

#include <graspit/EGPlanner/egPlanner.h>
#include <graspit/EGPlanner/energy/searchEnergy.h>
#include <graspit/EGPlanner/energy/searchEnergyFactory.h>
#include <graspit/EGPlanner/guidedPlanner.h>
#include <graspit/EGPlanner/searchState.h>
#include <graspit/EGPlanner/simAnnParams.h>
#include <graspit/EGPlanner/simAnnPlanner.h>
#include <graspit/bodySensor.h>
#include <graspit/cmdline/cmdline.h>
#include <graspit/grasp.h>
#include <graspit/quality/qualEpsilon.h>
#include <graspit/quality/qualVolume.h>
#include <graspit/quality/quality.h>

namespace GraspitInterface {

int GraspitInterface::init(int argc, char **argv) {
  cmdline::parser *parser = new cmdline::parser();

//   parser->add<std::string>("node_name", 'n', node_name_help, false);
  parser->parse(argc, argv);

  firstTimeInMainLoop = true;

  mPlanner = NULL;
  mHandObjectState = NULL;

  return 0;
}

int GraspitInterface::mainLoop() {
  if (firstTimeInMainLoop) {
    // Planner Must be started by mainthread, so it cannot be
    // Started inside the callback for the action server.  I need to connect
    // these here So that when the signal is emitted, the slot function is
    // executed by the correct thread.
    QObject::connect(this, SIGNAL(emitRunPlannerInMainThread()), this,
                     SLOT(runPlannerInMainThread()),
                     Qt::BlockingQueuedConnection);
    QObject::connect(this, SIGNAL(emitProcessPlannerResultsInMainThread()),
                     this, SLOT(processPlannerResultsInMainThread()),
                     Qt::BlockingQueuedConnection);
    QObject::connect(this, SIGNAL(emitBuildFeedbackInMainThread()), this,
                     SLOT(buildFeedbackInMainThread()),
                     Qt::BlockingQueuedConnection);
    firstTimeInMainLoop = false;

    runPlannerInMainThread();
  }

  if (!mPlanner->isActive()) {
    processPlannerResultsInMainThread();
    if (graspitCore->getMainWindow() == NULL) {
        // headless
        graspitCore->exitMainLoop();
    }
    std::cout << "DONE" << std::endl;
    return -1;
  }
  return 0;
}

/*
bool GraspitInterface::setRobotPoseCB(graspit_interface::SetRobotPose::Request
&request, graspit_interface::SetRobotPose::Response &response)
{
    if (graspitCore->getWorld()->getNumRobots() <= request.id) {
        response.result = response.RESULT_INVALID_ID;
        return true;
    } else {
        vec3 newTranslation(request.pose.position.x * 1000.0,
                            request.pose.position.y * 1000.0,
                            request.pose.position.z * 1000.0);

        Quaternion newRotation(request.pose.orientation.w,
                               request.pose.orientation.x,
                               request.pose.orientation.y,
                               request.pose.orientation.z);

        transf newTransform(newRotation, newTranslation);

        graspitCore->getWorld()->getRobot(request.id)->setTran(newTransform);
        return true;
    }
}

bool
GraspitInterface::setGraspableBodyPoseCB(graspit_interface::SetGraspableBodyPose::Request
&request, graspit_interface::SetGraspableBodyPose::Response &response)
{
    if (graspitCore->getWorld()->getNumGB() <= request.id) {
        response.result = response.RESULT_INVALID_ID;
        return true;
    } else {

        vec3 newTranslation(request.pose.position.x * 1000.0,
                            request.pose.position.y * 1000.0,
                            request.pose.position.z * 1000.0);

        Quaternion newRotation(request.pose.orientation.w,
                               request.pose.orientation.x,
                               request.pose.orientation.y,
                               request.pose.orientation.z);

        transf newTransform(newRotation, newTranslation);

        graspitCore->getWorld()->getGB(request.id)->setTran(newTransform);
        return true;
    }
}

bool GraspitInterface::setBodyPoseCB(graspit_interface::SetBodyPose::Request
&request, graspit_interface::SetBodyPose::Response &response)
{
    if (graspitCore->getWorld()->getNumBodies() <= request.id) {
        response.result = response.RESULT_INVALID_ID;
        return true;
    } else {

        vec3 newTranslation(request.pose.position.x * 1000.0,
                            request.pose.position.y * 1000.0,
                            request.pose.position.z * 1000.0);

        Quaternion newRotation(request.pose.orientation.w,
                               request.pose.orientation.x,
                               request.pose.orientation.y,
                               request.pose.orientation.z);

        transf newTransform(newRotation, newTranslation);

        graspitCore->getWorld()->getBody(request.id)->setTran(newTransform);
        return true;
    }
}

bool GraspitInterface::getDynamicsCB(graspit_interface::GetDynamics::Request
&request, graspit_interface::GetDynamics::Response &response)
{
    response.dynamicsEnabled = graspitCore->getWorld()->dynamicsAreOn();
    return true;
}

bool GraspitInterface::setDynamicsCB(graspit_interface::SetDynamics::Request
&request, graspit_interface::SetDynamics::Response &response)
{
    if(request.enableDynamics && (!graspitCore->getWorld()->dynamicsAreOn()))
    {
        graspitCore->getWorld()->turnOnDynamics();
        std::cout << "Turning Dynamics On" << std::endl;
    }
    else if((!request.enableDynamics) &&
graspitCore->getWorld()->dynamicsAreOn()){
        graspitCore->getWorld()->turnOffDynamics();
        std::cout << "Turning Dynamics Off" << std::endl;
    }
    return true;
}

bool GraspitInterface::autoGraspCB(graspit_interface::AutoGrasp::Request
&request, graspit_interface::AutoGrasp::Response &response)
{
    if (graspitCore->getWorld()->getNumRobots() <= request.id) {
        response.result = response.RESULT_INVALID_ID;
        return true;
    }
    else{
        graspitCore->getWorld()->getHand(request.id)->autoGrasp(true, 1.0,
false); graspitCore->getWorld()->updateGrasps();
    }
    return true;
}

bool GraspitInterface::autoOpenCB(graspit_interface::AutoOpen::Request &request,
                   graspit_interface::AutoOpen::Response &response)
{
    if (graspitCore->getWorld()->getNumRobots() <= request.id) {
        response.result = response.RESULT_INVALID_ID;
        return true;
    }
    else{
        graspitCore->getWorld()->getHand(request.id)->autoGrasp(true, -1.0,
false); graspitCore->getWorld()->updateGrasps();
    }
    return true;
}

bool GraspitInterface::forceRobotDOFCB(graspit_interface::ForceRobotDOF::Request
&request, graspit_interface::ForceRobotDOF::Response &response)
{
    if (graspitCore->getWorld()->getNumRobots() <= request.id) {
        response.result = response.RESULT_INVALID_ID;
        return true;
    } else if (graspitCore->getWorld()->dynamicsAreOn()) {
        response.result = response.RESULT_DYNAMICS_MODE_ENABLED;
        return true;
    } else {
        graspitCore->getWorld()->getHand(request.id)->forceDOFVals(request.dofs.data());
        response.result = response.RESULT_SUCCESS;
        return true;
    }
}

bool
GraspitInterface::moveDOFToContactsCB(graspit_interface::MoveDOFToContacts::Request
&request, graspit_interface::MoveDOFToContacts::Response &response)
{
    if (graspitCore->getWorld()->getNumRobots() <= request.id) {
        response.result = response.RESULT_INVALID_ID;
        return true;
    } else if (graspitCore->getWorld()->dynamicsAreOn()) {
        response.result = response.RESULT_DYNAMICS_MODE_ENABLED;
        return true;
    } else {
        graspitCore->getWorld()->getHand(request.id)->moveDOFToContacts(request.dofs.data(),
request.desired_steps.data(), request.stopAtContact); response.result =
response.RESULT_SUCCESS; return true;
    }
}

bool
GraspitInterface::setRobotDesiredDOFCB(graspit_interface::SetRobotDesiredDOF::Request
&request, graspit_interface::SetRobotDesiredDOF::Response &response)
{
    if (graspitCore->getWorld()->getNumRobots() <= request.id) {
        response.result = response.RESULT_INVALID_ID;
        return true;
    } else if (!graspitCore->getWorld()->dynamicsAreOn()) {
        response.result = response.RESULT_DYNAMICS_MODE_DISABLED;
        return true;
    } else {
        for(int i=0; i <
graspitCore->getWorld()->getHand(request.id)->getNumDOF(); i++) {
            graspitCore->getWorld()->getHand(request.id)->getDOF(i)->setDesiredVelocity(request.dof_velocities.data()[i]);
        }
        graspitCore->getWorld()->getHand(request.id)->setDesiredDOFVals(request.dofs.data());
        response.result = response.RESULT_SUCCESS;
        return true;
    }
}

bool GraspitInterface::importRobotCB(graspit_interface::ImportRobot::Request
&request, graspit_interface::ImportRobot::Response &response)
{
    QString filename = QString(getenv("GRASPIT"))+
            QString("/models/robots/") +
            QString(request.filename.data()) +
            QString("/") +
            QString(request.filename.data()) +
            QString(".xml");

    ROS_INFO("Loading %s",filename.toStdString().c_str());

    Robot * r = graspitCore->getWorld()->importRobot(filename);
    if(r == NULL){
        response.result = response.RESULT_FAILURE;
        return true;
    }
    vec3 newTranslation(request.pose.position.x * 1000.0,
                        request.pose.position.y * 1000.0,
                        request.pose.position.z * 1000.0);

    Quaternion newRotation(request.pose.orientation.w,
                           request.pose.orientation.x,
                           request.pose.orientation.y,
                           request.pose.orientation.z);

    transf newTransform(newRotation, newTranslation);
    r->setTran(newTransform);
    return true;
}

bool
GraspitInterface::importObstacleCB(graspit_interface::ImportObstacle::Request
&request, graspit_interface::ImportObstacle::Response &response)
{
    QString filename = QString(getenv("GRASPIT"))+
            QString("/models/obstacles/") +
            QString(request.filename.data()) +
            QString(".xml");

    ROS_INFO("Loading %s", filename.toStdString().c_str());

    Body * b = graspitCore->getWorld()->importBody(QString("Body"),filename);
    if(b == NULL){
        //Now try to load using unaltered filepath from request.
        b =
graspitCore->getWorld()->importBody(QString("Body"),QString(request.filename.data()));
        if(b == NULL){
            response.result = response.RESULT_FAILURE;
            return true;
        }
    }

    vec3 newTranslation(request.pose.position.x * 1000.0,
                        request.pose.position.y * 1000.0,
                        request.pose.position.z * 1000.0);

    Quaternion newRotation(request.pose.orientation.w,
                           request.pose.orientation.x,
                           request.pose.orientation.y,
                           request.pose.orientation.z);

    transf newTransform(newRotation, newTranslation);
    b->setTran(newTransform);
    return true;
}

bool
GraspitInterface::importGraspableBodyCB(graspit_interface::ImportGraspableBody::Request
&request, graspit_interface::ImportGraspableBody::Response &response)
{
    QString filename = QString(getenv("GRASPIT"))+
            QString("/models/objects/") +
            QString(request.filename.data()) +
            QString(".xml");

    ROS_INFO("Loading %s",filename.toStdString().c_str());
    //First try to load from Graspit Directory
    Body * b =
graspitCore->getWorld()->importBody(QString("GraspableBody"),filename); if(b ==
NULL){
        //Now try to load using unaltered filepath from request.
        b =
graspitCore->getWorld()->importBody(QString("GraspableBody"),QString(request.filename.data()));
        if(b == NULL){
            response.result = response.RESULT_FAILURE;
            return true;
        }
    }

    vec3 newTranslation(request.pose.position.x * 1000.0,
                        request.pose.position.y * 1000.0,
                        request.pose.position.z * 1000.0);

    Quaternion newRotation(request.pose.orientation.w,
                           request.pose.orientation.x,
                           request.pose.orientation.y,
                           request.pose.orientation.z);

    transf newTransform(newRotation, newTranslation);
    b->setTran(newTransform);

    return true;
}


bool GraspitInterface::loadWorldCB(graspit_interface::LoadWorld::Request
&request, graspit_interface::LoadWorld::Response &response)
{
    QString filename = QString(getenv("GRASPIT"))+
            QString("/worlds/") +
            QString(request.filename.data()) +
            QString(".xml");

    ROS_INFO("Loading World: %s",filename.toStdString().c_str());
    int result = graspitCore->getWorld()->load(filename);
    if(result == FAILURE){
        response.result = response.RESULT_FAILURE;
        return true;
    }
    return true;
}

bool GraspitInterface::saveWorldCB(graspit_interface::SaveWorld::Request
&request, graspit_interface::SaveWorld::Response &response)
{
    QString filename = QString(getenv("GRASPIT"))+
            QString("/worlds/") +
            QString(request.filename.data()) +
            QString(".xml");

    ROS_INFO("Saving World: %s",filename.toStdString().c_str());
    int result = graspitCore->getWorld()->save(filename);
    if(result == FAILURE){
        response.result = response.RESULT_FAILURE;
        return true;
    }
    return true;
}

bool GraspitInterface::clearWorldCB(graspit_interface::ClearWorld::Request
&request, graspit_interface::ClearWorld::Response &response)
{
    std::cout << "Emptying World" << std::endl;
    graspitCore->emptyWorld();

    return true;
}

bool GraspitInterface::saveImageCB(graspit_interface::SaveImage::Request
&request, graspit_interface::SaveImage::Response &response)
{
    QString filename = QString(getenv("GRASPIT"))+
            QString("/images/") +
            QString(request.filename.data()) +
            QString(".jpg");

    ROS_INFO("Saving Image: %s",filename.toStdString().c_str());
    graspitCore->getIVmgr()->saveImage(filename);
    return true;
}

bool
GraspitInterface::toggleAllCollisionsCB(graspit_interface::ToggleAllCollisions::Request
&request, graspit_interface::ToggleAllCollisions::Response &response)
{
    graspitCore->getWorld()->toggleAllCollisions(request.enableCollisions);
    if(request.enableCollisions)
    {
        std::cout << "Collision Detection is On, objects cannot interpentrate"
<< std::endl;
    }
    else
    {
        std::cout << "Collision Detection is Off, objects can interpentrate" <<
std::endl;
    }
    return true;
}

bool
GraspitInterface::computeQualityCB(graspit_interface::ComputeQuality::Request
&request, graspit_interface::ComputeQuality::Response &response)
{
    CollisionReport colReport;
    // first test whether the hand is in collision now
    int numCols = graspitCore->getWorld()->getCollisionReport(&colReport);
    // if it is in collision, then there should be no reason to calculate the
quality if(numCols>0){ response.result = response.RESULT_COLLISION;
        response.epsilon = -1.0;
        response.volume = -1.0;
        return true;
    }
    Hand *mHand =graspitCore->getWorld()->getHand(request.id);
    if (mHand==NULL)
    {
        response.result = response.RESULT_INVALID_ID;
        return true;
    }

    // if there is no collision, then begin computation

    QualVolume mVolQual( mHand->getGrasp(), ("Volume"),"L1 Norm");
    QualEpsilon mEpsQual( mHand->getGrasp(), ("Epsilon"),"L1 Norm");

    graspitCore->getWorld()->findAllContacts();
    graspitCore->getWorld()->updateGrasps();

    response.epsilon = mEpsQual.evaluate();
    response.volume = mVolQual.evaluate();

    return true;
}

bool GraspitInterface::computeEnergyCB(graspit_interface::ComputeEnergy::Request
&request, graspit_interface::ComputeEnergy::Response &response)
{
    Hand *mHand =graspitCore->getWorld()->getHand(request.handId);
    if (mHand==NULL)
    {
        response.result = response.RESULT_INVALID_HAND_ID;
        std::cout << "Planning Hand is NULL" << std::endl;
        return true;
    }
    GraspableBody *mObject =
graspitCore->getWorld()->getGB(request.graspableBodyId); if(mObject == NULL)
    {
        std::cout << "Planning Object is NULL" << std::endl;
        response.result = response.RESULT_INVALID_BODY_ID;
        return true;
    }

    graspitCore->getWorld()->findAllContacts();
    graspitCore->getWorld()->updateGrasps();

    std::vector<std::string> energyTypes =
SearchEnergyFactory::getInstance()->getAllRegisteredEnergy();
    if(std::find(energyTypes.begin(),energyTypes.end(), request.energyType) ==
energyTypes.end())
      {
        ROS_INFO_STREAM("Invalid Energy Type " << request.energyType <<
std::endl); response.result = response.RESULT_INVALID_ENERGY_TYPE; return true;
      }

    SearchEnergy *se =
SearchEnergyFactory::getInstance()->createEnergy(request.energyType);

    bool isLegal;
    double stateEnergy;
    se->analyzeCurrentPosture(mHand, mObject, isLegal, stateEnergy);
    response.isLegal = isLegal;
    response.energy= stateEnergy;

    return true;
}

bool
GraspitInterface::approachToContactCB(graspit_interface::ApproachToContact::Request
&request, graspit_interface::ApproachToContact::Response &response)
{
    Hand *mHand =graspitCore->getWorld()->getHand(request.id);
    if (mHand==NULL)
    {
        response.result = response.RESULT_INVALID_ID;
        return true;
    }
     mHand->approachToContact(request.moveDist, request.oneStep);
     return true;
}

bool
GraspitInterface::findInitialContactCB(graspit_interface::FindInitialContact::Request
&request, graspit_interface::FindInitialContact::Response &response)
{
     Hand *mHand =graspitCore->getWorld()->getHand(request.id);
     if (mHand==NULL)
     {
         response.result = response.RESULT_INVALID_ID;
         return true;
     }

     mHand->findInitialContact(request.moveDist);
     return true;
}

bool
GraspitInterface::dynamicAutoGraspCompleteCB(graspit_interface::DynamicAutoGraspComplete::Request
&request, graspit_interface::DynamicAutoGraspComplete::Response &response)
{
     Hand *mHand = graspitCore->getWorld()->getCurrentHand();
     if (mHand==NULL)
     {
         response.result = response.RESULT_INVALID_ID;
         return true;
     }
     response.GraspComplete = mHand->dynamicAutograspComplete();
     return true;
}
*/

/*
void GraspitInterface::PlanGraspsCB(
    const graspit_interface::PlanGraspsGoalConstPtr &_goal) {
  goal = *_goal;
    std::cout << "About to Call emit runPlannerInMainLoop( << std::endl;");
    emit emitRunPlannerInMainThread();

    std::cout << "Waiting For Planner to Finish" << std::endl;
    int last_feedback_step;
    while (mPlanner->isActive()) {
      if (_goal->feedback_num_steps < 1) {
        continue;
      }
      int current_feedback_step = mPlanner->getCurrentStep();
      if (current_feedback_step == mPlanner->getStartingStep()) {
        continue;
      }
      if ((current_feedback_step % _goal->feedback_num_steps == 0) &&
          (current_feedback_step != last_feedback_step)) {
        ROS_INFO("Curret Planner Step: %d", mPlanner->getCurrentStep());
        ROS_INFO("Curret Num Grasps: %d", mPlanner->getListSize());

        // collect grasps
        emit emitBuildFeedbackInMainThread();
        plan_grasps_as->publishFeedback(feedback_);
        last_feedback_step = current_feedback_step;
      }
    }

    std::cout << "About to Call emit emitProcessPlannerResultsInMainThread( <<
std::endl;"); emit emitProcessPlannerResultsInMainThread();

    plan_grasps_as->setSucceeded(result_);
    std::cout << "Action ServerCB Finished" << std::endl;
}
*/

/*
void GraspitInterface::buildFeedbackInMainThread() {
  feedback_.current_step = mPlanner->getCurrentStep();

  Hand *mHand = graspitCore->getWorld()->getCurrentHand();

  feedback_.grasps.clear();
  feedback_.energies.clear();
  for (int i = 0; i < mPlanner->getListSize(); i++) {
    const GraspPlanningState *gps = mPlanner->getGrasp(i);
    graspit_interface::Grasp g;
    graspPlanningStateToROSMsg(gps, g, mHand);

    feedback_.grasps.push_back(g);
    feedback_.energies.push_back(gps->getEnergy());
    feedback_.search_energy = goal.search_energy;
  }
}
*/

void GraspitInterface::runPlannerInMainThread() {
  std::cout << "Planner Starting in Mainloop" << std::endl;
  Hand *mHand = graspitCore->getWorld()->getCurrentHand();
  if (mHand == NULL) {
    std::cout << "Planning Hand is NULL" << std::endl;
  }
  GraspableBody *mObject = graspitCore->getWorld()->getGB(0);
  if (mObject == NULL) {
    std::cout << "Planning Object is NULL" << std::endl;
  }

  std::cout << "Initing mHandObjectState" << std::endl;
  mHandObjectState = new GraspPlanningState(mHand);
  mHandObjectState->setObject(mObject);

  switch (mRequest.search_space) {
  case SPACE_COMPLETE:
    mHandObjectState->setPositionType(SPACE_COMPLETE);
    mHandObjectState->setRefTran(mObject->getTran());
    break;
  case SPACE_AXIS_ANGLE:
    mHandObjectState->setPositionType(SPACE_AXIS_ANGLE);
    mHandObjectState->setRefTran(mObject->getTran());
    break;
  case SPACE_ELLIPSOID:
    mHandObjectState->setPositionType(SPACE_ELLIPSOID);
    mHandObjectState->setRefTran(mObject->getTran());
    break;
  case SPACE_APPROACH:
    mHandObjectState->setPositionType(SPACE_APPROACH);
    mHandObjectState->setRefTran(mHand->getTran());
    break;

  default:
    std::cout << "Invalid Search Space Type" << std::endl;
    return;
  }

  std::cout << "Initing mHandObjectState" << std::endl;
  mHandObjectState->reset();

  std::cout << "Initing mPlanner" << std::endl;

  switch (mRequest.planner) {
  case PLANNER_SIM_ANN:
    mPlanner = new SimAnnPlanner(mHand);
    std::cout << "Using graspit_interface::Planner::SIM_ANN " << std::endl;
    break;
  case PLANNER_MT:
    mPlanner = new GuidedPlanner(mHand);
    std::cout << "Using graspit_interface::Planner::MULTI_THREADED "
              << std::endl;
    break;
  default:
    std::cout << "Invalid Planner Type" << std::endl;
    return;
  }

  mPlanner->setEnergyType(mRequest.search_energy);

  //   if (goal.sim_ann_params.set_custom_params) {
  //     std::cout << "Switching SimAnn Annealing parameters to your custom
  //     defined "
  //                  "values!!! "
  //               << std::endl;
  //     SimAnnParams simAnnParams;
  //     simAnnParams.YC = goal.sim_ann_params.YC;
  //     simAnnParams.HC = goal.sim_ann_params.HC;
  //     simAnnParams.YDIMS = goal.sim_ann_params.YDIMS;
  //     simAnnParams.NBR_ADJ = goal.sim_ann_params.NBR_ADJ;
  //     simAnnParams.ERR_ADJ = goal.sim_ann_params.ERR_ADJ;
  //     simAnnParams.DEF_T0 = goal.sim_ann_params.DEF_T0;
  //     simAnnParams.DEF_K0 = goal.sim_ann_params.DEF_K0;
  //     mPlanner->setAnnealingParameters(simAnnParams);
  //   }

  switch (mRequest.search_contact) {
  case CONTACT_PRESET:
    mPlanner->setContactType(CONTACT_PRESET);
    std::cout << "Using graspit_interface::SearchContact::CONTACT_PRESET "
              << std::endl;
    break;
  case CONTACT_LIVE:
    mPlanner->setContactType(CONTACT_LIVE);
    std::cout << "Using graspit_interface::SearchContact::CONTACT_LIVE "
              << std::endl;
    break;
  default:
    std::cout << "Invalid Search Contact Type" << std::endl;
    return;
  }

  std::cout << "Setting Planner Model State" << std::endl;
  mPlanner->setModelState(mHandObjectState);
  int max_steps = mRequest.max_steps;
  if (max_steps == 0) {
    max_steps = 70000;
  }
  std::cout << "Setting Planner Max Steps " << max_steps << std::endl;
  mPlanner->setMaxSteps(max_steps);

  mPlanner->setBestListSizeParam(mRequest.max_num_results);

  std::cout << "resetting Planner" << std::endl;
  mPlanner->resetPlanner();

  std::cout << "Starting Planner" << std::endl;
  mPlanner->startPlanner();
}

void GraspitInterface::graspPlanningStateToCout(const GraspPlanningState *gps,
                                                  Hand *mHand,
                                                  GraspableBody *mObject) {
  gps->execute(mHand);
//   mHand->autoGrasp(false, 1.0, false);
  
  // Position of the object in the hand frame.
  transf t = mHand->getTran().inverse() % mObject->getTran();
  vec3 approachInHand = mHand->getApproachTran() * vec3(0, 0, 1);
  approachInHand.normalize();
  Eigen::VectorXd dof (mHand->getNumDOF());
  mHand->getDOFVals(dof.data());

  // This SEGV.
//   mHand->getGrasp()->update();
//   QualVolume mVolQual(mHand->getGrasp(), ("Volume"), "L1 Norm");
//   QualEpsilon mEpsQual(mHand->getGrasp(), ("Epsilon"), "L1 Norm");

//   graspitCore->getWorld()->findAllContacts();
//   graspitCore->getWorld()->updateGrasps();

  std::cout <<
  "T: " << t.translation().transpose() / 1000.0 << "\n"
  "R: " << t.rotation().coeffs().transpose() << "\n"
  "approach: " << approachInHand.transpose() << "\n"
  "dof: " << dof.transpose() << "\n"
//   "epsilon_quality" << mEpsQual.evaluate() << "\n"
//   "volume_quality" << mVolQual.evaluate() << "\n"
  ;
}

void GraspitInterface::processPlannerResultsInMainThread() {
  Hand *mHand = graspitCore->getWorld()->getCurrentHand();
  if (mHand == NULL) {
    std::cout << "Planning Hand is NULL" << std::endl;
  }
  GraspableBody *mObject = graspitCore->getWorld()->getGB(0);
  if (mObject == NULL) {
    std::cout << "Planning Object is NULL" << std::endl;
  }


  std::cout << "Publishing Result" << std::endl;
  for (int i = 0; i < mPlanner->getListSize(); i++) {
    const GraspPlanningState *gps = mPlanner->getGrasp(i);
    std::cout << "Grasp " << i << '\n';
    graspPlanningStateToCout(gps, mHand, mObject);
  }

  if (mPlanner->getListSize() > 0) {
    std::cout << "Showing Grasp 0" << std::endl;
    mPlanner->showGrasp(0);
  }

  if (mHandObjectState != NULL) {
    delete mHandObjectState;
    mHandObjectState = NULL;
  }

  if (mPlanner != NULL) {
    delete mPlanner;
    mPlanner = NULL;
  }
}

} // namespace GraspitInterface
