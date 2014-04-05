/* Include the controller definition */
#include "eyebot_circle.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* Function definitions for logging */
#include <argos3/core/utility/logging/argos_log.h>

/****************************************/
/****************************************/

/* Altitude to circle to move along the circle */
static const Real ALTITUDE = 3.0f;

/* Radius of the circle */
static const Real CIRCLE_RADIUS = 2.0f;

/* How many points the robot traverses to move along the circle */
static const UInt32 CIRCLE_WAYPOINTS = 20;

/* The angle between each waypoint */
static const CRadians CIRCLE_SLICE(2.0f * ARGOS_PI / CIRCLE_WAYPOINTS);

/* Tolerance for the distance to a target point to decide to do something else */
static const Real PROXIMITY_TOLERANCE = 0.01f;

/****************************************/
/****************************************/

CEyeBotCircle::CEyeBotCircle() :
   m_pcPosAct(NULL),
   m_pcPosSens(NULL),
   m_pcRABSens(NULL) {}

/****************************************/
/****************************************/

void CEyeBotCircle::Init(TConfigurationNode& t_node) {
   /*
    * Get sensor/actuator handles
    *
    * The passed string (ex. "quadrotor_position") corresponds to the
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
    * file at the <controllers><eyebot_circle><actuators> and
    * <controllers><eyebot_circle><sensors> sections. If you forgot to
    * list a device in the XML and then you request it here, an error
    * occurs.
    */
   m_pcPosAct  = GetActuator<CCI_QuadRotorPositionActuator>("quadrotor_position");
   m_pcPosSens = GetSensor  <CCI_PositioningSensor        >("positioning"       );
   m_pcRABSens = GetSensor  <CCI_RangeAndBearingSensor    >("range_and_bearing" );
   /*
    * Initialize the state variables of the behavior
    */
   Reset();
}

/****************************************/
/****************************************/

void CEyeBotCircle::ControlStep() {
   /* Get RAB message, if any */
   RLOG << "Message received: ";
   if(! m_pcRABSens->GetReadings().empty()) {
      m_psFBMsg = &(m_pcRABSens->GetReadings()[0]);
      LOG << *reinterpret_cast<const UInt64*>(m_psFBMsg->Data.ToCArray());
   }
   else {
      m_psFBMsg = NULL;
      LOG << "none";
   }
   LOG << std::endl;
   /* Execute state logic */
   switch(m_eState) {
      case STATE_START:
         TakeOff();
         break;
      case STATE_TAKE_OFF:
         TakeOff();
         break;
      case STATE_LEAVE_CIRCLE_CENTER:
         LeaveCircleCenter();
         break;
      case STATE_MOVE_ALONG_CIRCLE:
         MoveAlongCircle();
         break;
      case STATE_GO_TO_CENTER:
         GoToCenter();
         break;
      case STATE_LAND:
         Land();
         break;
      default:
         LOGERR << "[BUG] Unknown robot state: " << m_eState << std::endl;
   }
   /* Write debug information */
   RLOG << "Current state: " << m_eState << std::endl;
   RLOG << "Target pos: " << m_cTargetPos << std::endl;
}

/****************************************/
/****************************************/

void CEyeBotCircle::Reset() {
   /* Start the behavior */
   m_eState = STATE_START;
   /* No message received */
   m_psFBMsg = NULL;
}

/****************************************/
/****************************************/

void CEyeBotCircle::TakeOff() {
   if(m_eState != STATE_TAKE_OFF) {
      /* State initialization */
      m_eState = STATE_TAKE_OFF;
      m_cCircleCenter = m_pcPosSens->GetReading().Position + CVector3(0.0f, 0.0f, ALTITUDE);
      m_cTargetPos = m_cCircleCenter;
      m_pcPosAct->SetAbsolutePosition(m_cTargetPos);
   }
   else {
      if(Distance(m_cTargetPos, m_pcPosSens->GetReading().Position) < PROXIMITY_TOLERANCE) {
         /* State transition */
         LeaveCircleCenter();
      }
   }
}

/****************************************/
/****************************************/

void CEyeBotCircle::LeaveCircleCenter() {
   if(m_eState != STATE_LEAVE_CIRCLE_CENTER) {
      /* State initialization */
      m_eState = STATE_LEAVE_CIRCLE_CENTER;
      m_cTargetPos = m_pcPosSens->GetReading().Position + CVector3(CIRCLE_RADIUS, 0.0f, 0.0f);
      m_pcPosAct->SetAbsolutePosition(m_cTargetPos);
   }
   else {
      if(Distance(m_cTargetPos, m_pcPosSens->GetReading().Position) < PROXIMITY_TOLERANCE) {
         /* State transition */
         MoveAlongCircle();
      }
   }
}

/****************************************/
/****************************************/

void CEyeBotCircle::MoveAlongCircle() {
   if(m_eState != STATE_MOVE_ALONG_CIRCLE) {
      /* State initialization */
      m_eState = STATE_MOVE_ALONG_CIRCLE;
      m_unWaypoint = 0;
   }
   else {
      if(m_unWaypoint >= CIRCLE_WAYPOINTS) {
         /* State transition */
         GoToCenter();
      }
      else {
         /* State logic */
         m_cTargetPos.Set(
            CIRCLE_RADIUS * Cos(CIRCLE_SLICE * m_unWaypoint),
            CIRCLE_RADIUS * Sin(CIRCLE_SLICE * m_unWaypoint),
            0.0f);
         m_cTargetPos += m_cCircleCenter;
         m_pcPosAct->SetAbsolutePosition(m_cTargetPos);
         ++m_unWaypoint;
      }
   }
}

/****************************************/
/****************************************/

void CEyeBotCircle::GoToCenter() {
   if(m_eState != STATE_GO_TO_CENTER) {
      /* State initialization */
      m_eState = STATE_GO_TO_CENTER;
      m_cTargetPos = m_cCircleCenter;
      m_pcPosAct->SetAbsolutePosition(m_cTargetPos);
   }
   else {
      if(Distance(m_cTargetPos, m_pcPosSens->GetReading().Position) < PROXIMITY_TOLERANCE) {
         /* State transition */
         Land();
      }
   }
}

/****************************************/
/****************************************/

void CEyeBotCircle::Land() {
   if(m_eState != STATE_LAND) {
      /* State initialization */
      m_eState = STATE_LAND;
      m_cTargetPos = m_pcPosSens->GetReading().Position;
      m_cTargetPos.SetZ(0.0f);
      m_pcPosAct->SetAbsolutePosition(m_cTargetPos);
   }
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
REGISTER_CONTROLLER(CEyeBotCircle, "eyebot_circle_controller")
