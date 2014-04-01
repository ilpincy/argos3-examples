/* Include the controller definition */
#include "footbot_circle.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* Function definitions for logging */
#include <argos3/core/utility/logging/argos_log.h>

/****************************************/
/****************************************/

CFootBotCircle::CFootBotCircle() :
   m_pcRABAct(NULL) {}

/****************************************/
/****************************************/

void CFootBotCircle::Init(TConfigurationNode& t_node) {
   /*
    * Get sensor/actuator handles
    *
    * The passed string (ex. "range_and_bearing") corresponds to the
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
    * file at the <controllers><footbot_circle><actuators> and
    * <controllers><footbot_circle><sensors> sections. If you forgot to
    * list a device in the XML and then you request it here, an error
    * occurs.
    */
   m_pcRABAct  = GetActuator<CCI_RangeAndBearingActuator  >("range_and_bearing");
}

/****************************************/
/****************************************/

void CFootBotCircle::ControlStep() {
   /* Send counter value */
   CByteArray cBuf(10);
   cBuf[0] = m_unCounter       & 0xff;
   cBuf[1] = m_unCounter >> 8  & 0xff;
   cBuf[2] = m_unCounter >> 16 & 0xff;
   cBuf[3] = m_unCounter >> 24 & 0xff;
   m_pcRABAct->SetData(cBuf);
   /* Write on robot log the sent value */
   RLOG << "Sent: " << m_unCounter << std::endl;
   /* Increase counter */
   ++m_unCounter;
}

/****************************************/
/****************************************/

void CFootBotCircle::Reset() {
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
REGISTER_CONTROLLER(CFootBotCircle, "footbot_circle_controller")
