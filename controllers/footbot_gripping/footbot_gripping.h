/*
 * AUTHOR: Carlo Pinciroli <cpinciro@ulb.ac.be>
 *
 * An example gripping controller for the foot-bot.
 *
 * The controller uses the the wheels to move the robot around, and the
 * gripper to transport the target object (a cylinder). The logic of this
 * controller is simple:
 * 1. the robot moves forward for some steps, until it finds the object;
 * 2. the robot closes the gripper;
 * 3. the robot moves backwards, dragging the object;
 * 4. the robot releases the object while keeping its backwards motion.
 *
 * This controller is meant to be used with the ARGoS file:
 *    experiments/gripping.argos
 */

#ifndef FOOTBOT_GRIPPING_H
#define FOOTBOT_GRIPPING_H

/*
 * Include some necessary headers.
 */
/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
/* Definition of the differential steering actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
/* Definition of the foot-bot gripper actuator */
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_gripper_actuator.h>

/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class CFootBotGripping : public CCI_Controller {

public:

   /* Class constructor. */
   CFootBotGripping();

   /* Class destructor. */
   virtual ~CFootBotGripping() {}

   /*
    * This function initializes the controller.
    * The 't_node' variable points to the <parameters> section in the XML
    * file in the <controllers><footbot_gripping_controller> section.
    */
   virtual void Init(TConfigurationNode& t_node);

   /*
    * This function is called once every time step.
    * The length of the time step is set in the XML file.
    */
   virtual void ControlStep();

   /*
    * This function resets the controller to its state right after the
    * Init().
    * It is called when you press the reset button in the GUI.
    * In this example controller there is no need for resetting anything,
    * so the function could have been omitted. It's here just for
    * completeness.
    */
   virtual void Reset();

   /*
    * Called to cleanup what done by Init() when the experiment finishes.
    * In this example controller there is no need for clean anything up,
    * so the function could have been omitted. It's here just for
    * completeness.
    */
   virtual void Destroy() {}

private:

   /* Pointer to the differential steering actuator */
   CCI_DifferentialSteeringActuator* m_pcWheels;
   /* Pointer to the foot-bot gripper actuator */
   CCI_FootBotGripperActuator* m_pcGripper;

   /* A counter used to know when to trigger each action */
   UInt64 m_unCounter;
};

#endif
