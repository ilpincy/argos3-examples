#ifndef MPGA_PHOTOTAXIS_LOOP_FUNCTIONS_H
#define MPGA_PHOTOTAXIS_LOOP_FUNCTIONS_H

/* The NN controller */
#include <controllers/footbot_nn/footbot_nn_controller.h>

/* ARGoS-related headers */
#include <argos3/core/utility/math/rng.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>

#include <loop_functions/mpga_loop_functions/mpga_loop_functions.h>

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

class CMPGAPhototaxisLoopFunctions : public CMPGALoopFunctions {

public:

   CMPGAPhototaxisLoopFunctions();
   virtual ~CMPGAPhototaxisLoopFunctions();

   virtual void Init(TConfigurationNode& t_node);
   virtual void Reset();

   /* Configures the robot controller from the genome */
   virtual void ConfigureFromGenome(const Real* pf_genome);

   /* Calculates the performance of the robot in a trial */
   virtual Real Score();

private:

   /* The initial setup of a trial */
   struct SInitSetup {
      CVector3 Position;
      CQuaternion Orientation;
   };

   std::vector<SInitSetup> m_vecInitSetup;
   CFootBotEntity* m_pcFootBot;
   CFootBotNNController* m_pcController;
   Real* m_pfControllerParams;
   CRandom::CRNG* m_pcRNG;


};

#endif
