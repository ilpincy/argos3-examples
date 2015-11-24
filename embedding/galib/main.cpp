/* GA-related headers */
#include <ga/ga.h>

/* ARGoS-related headers */
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/loop_functions.h>

#include <loop_functions/galib_phototaxis_loop_functions/galib_phototaxis_loop_functions.h>

/****************************************/
/****************************************/

/*
 * Launch ARGoS to evaluate a genome.
 */
float LaunchARGoS(GAGenome& c_genome) {
   /* Convert the received genome to the actual genome type */
   GARealGenome& cRealGenome = dynamic_cast<GARealGenome&>(c_genome);
   /* The CSimulator class of ARGoS is a singleton. Therefore, to
    * manipulate an ARGoS experiment, it is enough to get its instance.
    * This variable is declared 'static' so it is created
    * once and then reused at each call of this function.
    * This line would work also without 'static', but written this way
    * it is faster. */
   static argos::CSimulator& cSimulator = argos::CSimulator::GetInstance();
   /* Get a reference to the loop functions */
   static CGALibPhototaxisLoopFunctions& cLoopFunctions = dynamic_cast<CGALibPhototaxisLoopFunctions&>(cSimulator.GetLoopFunctions());
   /*
    * Run 5 trials and take the worst performance as final value.
    * Performance in this experiment is defined as the distance from the light.
    * Thus, we keep the max distance found.
    */
   Real fDistance = 0.0f;
   for(size_t i = 0; i < 5; ++i) {
      /* Tell the loop functions to get ready for the i-th trial */
      cLoopFunctions.SetTrial(i);
      /* Reset the experiment.
       * This internally calls also CEvolutionLoopFunctions::Reset(). */
      cSimulator.Reset();
      /* Configure the controller with the genome */
      cLoopFunctions.ConfigureFromGenome(cRealGenome);
      /* Run the experiment */
      cSimulator.Execute();
      /* Update performance */
      fDistance = Max(fDistance, cLoopFunctions.Performance());
   }
   /* Return the result of the evaluation */
   return fDistance;
}

/*
 * Flush best individual
 */
void FlushBest(const GARealGenome& c_genome,
               size_t un_generation) {
   std::ostringstream cOSS;
   cOSS << "best_" << un_generation << ".dat";
   std::ofstream cOFS(cOSS.str().c_str(), std::ios::out | std::ios::trunc);
   cOFS << GENOME_SIZE // first write the number of values to dump
        << " "
        << c_genome    // then write the actual values
        << std::endl;
}

/****************************************/
/****************************************/

int main(int argc, char** argv) {
   /*
    * Initialize GALIB
    */
   /* Create an allele whose values can be in the range [-10,10] */
   GAAlleleSet<float> cAlleleSet(-10.0f, 10.0f);
   /* Create a genome with 10 genes, using LaunchARGoS() to evaluate it */
   GARealGenome cGenome(GENOME_SIZE, cAlleleSet, LaunchARGoS);
   /* Create and configure a basic genetic algorithm using the genome */
   GASimpleGA cGA(cGenome);
   cGA.minimize();                     // the objective function must be minimized
   cGA.populationSize(5);              // population size for each generation
   cGA.nGenerations(500);              // number of generations
   cGA.pMutation(0.05f);               // prob of gene mutation
   cGA.pCrossover(0.15f);              // prob of gene crossover
   cGA.scoreFilename("evolution.dat"); // filename for the result log
   cGA.flushFrequency(1);              // log the results every generation

   /*
    * Initialize ARGoS
    */
   /* The CSimulator class of ARGoS is a singleton. Therefore, to
    * manipulate an ARGoS experiment, it is enough to get its instance */
   argos::CSimulator& cSimulator = argos::CSimulator::GetInstance();
   /* Set the .argos configuration file
    * This is a relative path which assumed that you launch the executable
    * from argos3-examples (as said also in the README) */
   cSimulator.SetExperimentFileName("experiments/galib.argos");
   /* Load it to configure ARGoS */
   cSimulator.LoadExperiment();

   /*
    * Launch the evolution, setting the random seed
    */
   cGA.initialize(12345);
   do {
      argos::LOG << "Generation #" << cGA.generation() << "...";
      cGA.step();
      argos::LOG << "done.";
      if(cGA.generation() % cGA.flushFrequency() == 0) {
         argos::LOG << "   Flushing...";
         /* Flush scores */
         cGA.flushScores();
         /* Flush best individual */
         FlushBest(dynamic_cast<const GARealGenome&>(cGA.statistics().bestIndividual()),
                   cGA.generation());
         argos::LOG << "done.";
      }
      LOG << std::endl;
      LOG.Flush();
   }
   while(! cGA.done());

   /*
    * Dispose of ARGoS stuff
    */
   cSimulator.Destroy();

   /* All is OK */
   return 0;
}

/****************************************/
/****************************************/
