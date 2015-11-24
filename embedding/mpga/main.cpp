/*
 * This is a simple example of a multi-process genetic algorithm that
 * uses multiple processes to parallelize the optimization process.
 */

#include <iostream>
#include <fstream>
#include <loop_functions/mpga_loop_functions/mpga.h>
#include <loop_functions/mpga_loop_functions/mpga_phototaxis_loop_functions.h>

/*
 * Flush best individual
 */
void FlushIndividual(const CMPGA::SIndividual& s_ind,
                     UInt32 un_generation) {
   std::ostringstream cOSS;
   cOSS << "best_" << un_generation << ".dat";
   std::ofstream cOFS(cOSS.str().c_str(), std::ios::out | std::ios::trunc);
   /* First write the number of values to dump */
   cOFS << GENOME_SIZE;
   /* Then dump the genome */
   for(UInt32 i = 0; i < GENOME_SIZE; ++i) {
      cOFS << " " << s_ind.Genome[i];
   }
   /* End line */
   cOFS << std::endl;
}

/*
 * The function used to aggregate the scores of each trial.  In this
 * experiment, the score is the distance of the robot from the
 * light. We take the maximum value as aggregated score.
 */
Real ScoreAggregator(const std::vector<Real>& vec_scores) {
   Real fScore = vec_scores[0];
   for(size_t i = 1; i < vec_scores.size(); ++i) {
      fScore = Max(fScore, vec_scores[i]);
   }
   return fScore;
}

int main() {
   CMPGA cGA(CRange<Real>(-10.0,10.0),            // Allele range
             GENOME_SIZE,                         // Genome size
             5,                                   // Population size
             0.05,                                // Mutation probability
             5,                                   // Number of trials
             100,                                 // Number of generations
             false,                               // Minimize score
             "experiments/mpga.argos",            // .argos conf file
             &ScoreAggregator,                    // The score aggregator
             12345                                // Random seed
      );
   cGA.Evaluate();
   argos::LOG << "Generation #" << cGA.GetGeneration() << "...";
   argos::LOG << " scores:";
   for(UInt32 i = 0; i < cGA.GetPopulation().size(); ++i) {
      argos::LOG << " " << cGA.GetPopulation()[i]->Score;
   }
   LOG << std::endl;
   LOG.Flush();
   while(!cGA.Done()) {
      cGA.NextGen();
      cGA.Evaluate();
      argos::LOG << "Generation #" << cGA.GetGeneration() << "...";
      argos::LOG << " scores:";
      for(UInt32 i = 0; i < cGA.GetPopulation().size(); ++i) {
         argos::LOG << " " << cGA.GetPopulation()[i]->Score;
      }
      if(cGA.GetGeneration() % 5 == 0) {
         argos::LOG << " [Flushing genome... ";
         /* Flush scores of best individual */
         FlushIndividual(*cGA.GetPopulation()[0],
                         cGA.GetGeneration());
         argos::LOG << "done.]";
      }
      LOG << std::endl;
      LOG.Flush();
   }
   return 0;
}
