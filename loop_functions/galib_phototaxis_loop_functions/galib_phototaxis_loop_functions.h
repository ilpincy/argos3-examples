#ifndef GALIB_PHOTOTAXIS_LOOP_FUNCTIONS_H
#define GALIB_PHOTOTAXIS_LOOP_FUNCTIONS_H


/* The NN controller */
#include <controllers/footbot_nn/footbot_nn_controller.h>

/* ARGoS-related headers */
#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>

/* GA-related headers */
#include <ga/ga.h>
#include <ga/GARealGenome.h>
#include <ga/GARealGenome.C> // this is necessary!

/****************************************/
/****************************************/

/*
 * The size of the genome.
 * 
 * The genome is the set of NN weights. The NN is a simple
 * 2-layer perceptron. The inputs are 24 proximity readings and
 * 24 light readings. The outputs are 2 wheels speeds. The total
 * number of weights is therefore:
 *
 * W = (I + 1) * O = (24 + 24 + 1) * 2 = 98
 *
 * where:
 *   W = number of weights
 *   I = number of inputs
 *   O = number of outputs
 */
static const size_t GENOME_SIZE = 98;

/****************************************/
/****************************************/

using namespace argos;

class CGALibPhototaxisLoopFunctions : public CLoopFunctions {

public:

   CGALibPhototaxisLoopFunctions();
   virtual ~CGALibPhototaxisLoopFunctions();

   virtual void Init(TConfigurationNode& t_node);
   virtual void Reset();

   /* Called by the evolutionary algorithm to set the current trial */
   inline void SetTrial(size_t un_trial) {
      m_unCurrentTrial = un_trial;
   }

   /* Configures the robot controller from the genome */
   void ConfigureFromGenome(const GARealGenome& c_genome);

   /* Calculates the performance of the robot in a trial */
   Real Performance();

private:

   /* The initial setup of a trial */
   struct SInitSetup {
      CVector3 Position;
      CQuaternion Orientation;
   };

   size_t m_unCurrentTrial;
   std::vector<SInitSetup> m_vecInitSetup;
   CFootBotEntity* m_pcFootBot;
   CFootBotNNController* m_pcController;
   Real* m_pfControllerParams;
   CRandom::CRNG* m_pcRNG;


};

#endif
