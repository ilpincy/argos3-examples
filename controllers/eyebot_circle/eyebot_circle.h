#ifndef EYEBOT_CIRCLE_H
#define EYEBOT_CIRCLE_H

/*
 * Include some necessary headers.
 */
/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
/* Definition of the quadrotor positioning actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_quadrotor_position_actuator.h>
/* Definition of the positioning sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
/* Definition of the range-and-bearing sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>

/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class CEyeBotCircle : public CCI_Controller {

public:

   /* Class constructor. */
   CEyeBotCircle();

   /* Class destructor. */
   virtual ~CEyeBotCircle() {}

   /*
    * This function initializes the controller.
    * The 't_node' variable points to the <parameters> section in the XML
    * file in the <controllers><eyebot_circle_controller> section.
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

   /*
    * Takes off the robot.
    */
   void TakeOff();

   /*
    * Moves the robot towards the circle.
    */
   void LeaveCircleCenter();

   /*
    * Moves the robot along the circle.
    */
   void MoveAlongCircle();

   /*
    * Moves the robot to the center.
    */
   void GoToCenter();

   /*
    * Lands the robot.
    */
   void Land();

private:

   /* Current robot state */
   enum EState {
      STATE_START = 0,
      STATE_TAKE_OFF,
      STATE_LEAVE_CIRCLE_CENTER,
      STATE_MOVE_ALONG_CIRCLE,
      STATE_GO_TO_CENTER,
      STATE_LAND
   };

private:

   /* Pointer to the quadrotor position actuator */
   CCI_QuadRotorPositionActuator* m_pcPosAct;
   /* Pointer to the positioning sensor */
   CCI_PositioningSensor* m_pcPosSens;
   /* Pointer to the range-and-bearing sensor */
   CCI_RangeAndBearingSensor* m_pcRABSens;

   /* Current robot state */
   EState m_eState;
   /* Circle center */
   CVector3 m_cCircleCenter;
   /* Current target position */
   CVector3 m_cTargetPos;
   /* Used to move the robot along the circle */
   UInt32 m_unWaypoint;

   /* Contains the message received from the foot-bot */
   const CCI_RangeAndBearingSensor::SPacket* m_psFBMsg;
};

#endif
