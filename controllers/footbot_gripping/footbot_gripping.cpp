/* Include the controller definition */
#include "footbot_gripping.h"

/****************************************/
/****************************************/

CFootBotGripping::CFootBotGripping() :
   m_pcWheels(NULL),
   m_pcGripper(NULL),
   m_unCounter(0) {}

/****************************************/
/****************************************/

void CFootBotGripping::Init(TConfigurationNode& t_node) {
   /*
    * Get sensor/actuator handles
    *
    * The passed string (ex. "differential_steering") corresponds to the
    * XML tag of the device whose handle we want to have. For a list of
    * allowed values, type at the command prompt:
    *
    * $ argos3 -q actuators
    *
    * to have a list of all the possible actuators, or
    *
    * $ argos3 -q sensors
    *
    * to have a list of all the possible sensors.
    *
    * NOTE: ARGoS creates and initializes actuators and sensors
    * internally, on the basis of the lists provided the configuration
    * file at the <controllers><footbot_gripping><actuators> and
    * <controllers><footbot_gripping><sensors> sections. If you forgot to
    * list a device in the XML and then you request it here, an error
    * occurs.
    */
   m_pcWheels  = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
   m_pcGripper = GetActuator<CCI_FootBotGripperActuator      >("footbot_gripper"      );
}

/****************************************/
/****************************************/

void CFootBotGripping::ControlStep() {
   /* Increase the counter */
   ++m_unCounter;
   /* Depending on the value of the counter, pick an action */
   if(m_unCounter < 70) {
      /* Move forward to reach the object */
      m_pcWheels->SetLinearVelocity(5.0f, 5.0f);
   }
   else if(m_unCounter == 70) {
      /* Stop and grip the object */
      m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
      m_pcGripper->LockPositive();
   }
   else if(m_unCounter < 120) {
      /* Move backwards dragging the object */
      m_pcWheels->SetLinearVelocity(-5.0f, -5.0f);
   }
   else if(m_unCounter < 170) {
      /* Release the object and keep the backwards motion */
      m_pcGripper->Unlock();
   }
   else if(m_unCounter >= 170) {
      /* Stop forever */
      m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
   }
}

/****************************************/
/****************************************/

void CFootBotGripping::Reset() {
   /* Reset the counter */
   m_unCounter = 0;
}

/****************************************/
/****************************************/

/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as
 * second argument.
 * The string is then usable in the configuration file to refer to this
 * controller.
 * When ARGoS reads that string in the configuration file, it knows which
 * controller class to instantiate.
 * See also the configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(CFootBotGripping, "footbot_gripping_controller")
