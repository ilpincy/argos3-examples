#include "trajectory_qtuser_functions.h"
#include "trajectory_loop_functions.h"

/****************************************/
/****************************************/

CTrajectoryQTUserFunctions::CTrajectoryQTUserFunctions() :
   m_cTrajLF(dynamic_cast<CTrajectoryLoopFunctions&>(CSimulator::GetInstance().GetLoopFunctions())) {
}

/****************************************/
/****************************************/

void CTrajectoryQTUserFunctions::DrawInWorld() {
   /* Go through all the robot waypoints and draw them */
   for(CTrajectoryLoopFunctions::TWaypointMap::const_iterator it = m_cTrajLF.GetWaypoints().begin();
       it != m_cTrajLF.GetWaypoints().end();
       ++it) {
      DrawWaypoints(it->second);
   }
}

/****************************************/
/****************************************/

void CTrajectoryQTUserFunctions::DrawWaypoints(const std::vector<CVector3>& c_waypoints) {
   /* Start drawing segments when you have at least two points */
   if(c_waypoints.size() > 1) {
      size_t unStart = 0;
      size_t unEnd = 1;
      while(unEnd < c_waypoints.size()) {
         DrawRay(CRay3(c_waypoints[unEnd],
                       c_waypoints[unStart]));
         ++unStart;
         ++unEnd;
      }
   }
}

/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CTrajectoryQTUserFunctions, "trajectory_qtuser_functions")
