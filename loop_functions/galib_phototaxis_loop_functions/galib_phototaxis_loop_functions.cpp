#include "galib_phototaxis_loop_functions.h"

/****************************************/
/****************************************/

CGALibPhototaxisLoopFunctions::CGALibPhototaxisLoopFunctions() :
   m_unCurrentTrial(0),
   m_vecInitSetup(5),
   m_pcFootBot(NULL),
   m_pcController(NULL),
   m_pfControllerParams(new Real[GENOME_SIZE]),
   m_pcRNG(NULL) {}

/****************************************/
/****************************************/

CGALibPhototaxisLoopFunctions::~CGALibPhototaxisLoopFunctions() {
   delete[] m_pfControllerParams;
}

/****************************************/
/****************************************/

void CGALibPhototaxisLoopFunctions::Init(TConfigurationNode& t_node) {
   /*
    * Create the random number generator
    */
   m_pcRNG = CRandom::CreateRNG("argos");

   /*
    * Create the foot-bot and get a reference to its controller
    */
   m_pcFootBot = new CFootBotEntity(
      "fb",    // entity id
      "fnn"    // controller id as set in the XML
      );
   AddEntity(*m_pcFootBot);
   m_pcController = &dynamic_cast<CFootBotNNController&>(m_pcFootBot->GetControllableEntity().GetController());

   /*
    * Create the initial setup for each trial
    * The robot is placed 4.5 meters away from the light
    * (which is in the origin) at angles
    * { PI/12, 2*PI/12, 3*PI/12, 4*PI/12, 5*PI/12 }
    * wrt to the world reference.
    * Also, the rotation of the robot is chosen at random
    * from a uniform distribution.
    */
   CRadians cOrient;
   for(size_t i = 0; i < 5; ++i) {
      /* Set position */
      m_vecInitSetup[i].Position.FromSphericalCoords(
         4.5f,                                          // distance from origin
         CRadians::PI_OVER_TWO,                         // angle with Z axis
         static_cast<Real>(i+1) * CRadians::PI / 12.0f // rotation around Z
         );
      /* Set orientation */
      cOrient = m_pcRNG->Uniform(CRadians::UNSIGNED_RANGE);
      m_vecInitSetup[i].Orientation.FromEulerAngles(
         cOrient,        // rotation around Z
         CRadians::ZERO, // rotation around Y
         CRadians::ZERO  // rotation around X
         );
   }

   /*
    * Process trial information, if any
    */
   try {
      GetNodeAttribute(t_node, "trial", m_unCurrentTrial);
      Reset();
   }
   catch(CARGoSException& ex) {}
}

/****************************************/
/****************************************/

void CGALibPhototaxisLoopFunctions::Reset() {
   /*
    * Move robot to the initial position corresponding to the current trial
    */
   if(!MoveEntity(
         m_pcFootBot->GetEmbodiedEntity(),             // move the body of the robot
         m_vecInitSetup[m_unCurrentTrial].Position,    // to this position
         m_vecInitSetup[m_unCurrentTrial].Orientation, // with this orientation
         false                                         // this is not a check, leave the robot there
         )) {
      LOGERR << "Can't move robot in <"
             << m_vecInitSetup[m_unCurrentTrial].Position
             << ">, <"
             << m_vecInitSetup[m_unCurrentTrial].Orientation
             << ">"
             << std::endl;
   }
}

/****************************************/
/****************************************/

void CGALibPhototaxisLoopFunctions::ConfigureFromGenome(const GARealGenome& c_genome) {
   /* Copy the genes into the NN parameter buffer */
   for(size_t i = 0; i < GENOME_SIZE; ++i) {
      m_pfControllerParams[i] = c_genome[i];
   }
   /* Set the NN parameters */
   m_pcController->GetPerceptron().SetOnlineParameters(GENOME_SIZE, m_pfControllerParams);
}

/****************************************/
/****************************************/

Real CGALibPhototaxisLoopFunctions::Performance() {
   /* The performance is simply the distance of the robot to the origin */
   return m_pcFootBot->GetEmbodiedEntity().GetOriginAnchor().Position.Length();
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CGALibPhototaxisLoopFunctions, "galib_phototaxis_loop_functions")
