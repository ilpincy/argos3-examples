#include "mpga.h"
#include <cstdio>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <signal.h>
#include <iostream>
#include <fstream>
#include <argos3/core/simulator/simulator.h>
#include "mpga_loop_functions.h"

/****************************************/
/****************************************/

/* File name for shared memory area */
static const std::string SHARED_MEMORY_FILE = "/MPGA_SHARED_MEMORY_" + ToString(getpid());

/****************************************/
/****************************************/

bool SortHighToLow(const CMPGA::SIndividual* pc_a,
                   const CMPGA::SIndividual* pc_b) {
   return pc_a->Score > pc_b->Score;
}

bool SortLowToHigh(const CMPGA::SIndividual* pc_a,
                   const CMPGA::SIndividual* pc_b) {
   return pc_b->Score > pc_a->Score;
}

/****************************************/
/****************************************/

CMPGA::CMPGA(const CRange<Real>& c_allele_range,
             UInt32 un_genome_size,
             UInt32 un_pop_size,
             Real f_mutation_prob,
             UInt32 un_num_trials,
             UInt32 un_generations,
             bool b_maximize,
             const std::string& str_argosconf,
             TScoreAggregator t_score_aggregator,
             UInt32 un_random_seed) :
   m_unCurrentGeneration(0),
   m_cAlleleRange(c_allele_range),
   m_unGenomeSize(un_genome_size),
   m_unPopSize(un_pop_size),
   m_fMutationProb(f_mutation_prob),
   m_unNumTrials(un_num_trials),
   m_unGenerations(un_generations),
   m_strARGoSConf(str_argosconf),
   m_tScoreAggregator(t_score_aggregator),
   MasterPID(::getpid()),
   m_cIndComparator(b_maximize ? SortHighToLow : SortLowToHigh) {
   /* Create shared memory manager */
   m_pcSharedMem = new CSharedMem(un_genome_size,
                                  un_pop_size);
   /* Create slave processes */
   for(UInt32 i = 0; i < m_unPopSize; ++i) {
      /* Perform fork */
      SlavePIDs.push_back(::fork());
      if(SlavePIDs.back() == 0) {
         /* We're in a slave */
         LaunchARGoS(i);
      }
   }
   /* Create a random number generator */
   CRandom::CreateCategory("ga", un_random_seed);
   m_pcRNG = CRandom::CreateRNG("ga");
   /* Create initial population */
   SIndividual* psInd;
   for(size_t p = 0; p < m_unPopSize; ++p) {
      /* Create individual */
      psInd = new SIndividual;
      psInd->Score = -1.0;
      /* Create random genome */
      for(size_t g = 0; g < m_unGenomeSize; ++g) {
         psInd->Genome.push_back(m_pcRNG->Uniform(m_cAlleleRange));
      }
      /* Add individual to the population */
      m_tPopulation.push_back(psInd);
   }
   /* The master sleeps to give enough time to the slaves to
    * initialize and suspend properly. If not enough time is given
    * here, the master will hang later on. */
   ::sleep(3);
}

/****************************************/
/****************************************/

CMPGA::~CMPGA() {
   /* Terminate slaves */
   for(UInt32 i = 0; i < m_unPopSize; ++i) {
      ::kill(SlavePIDs[i], SIGTERM);
   }
   /* Clean memory up */
   while(!m_tPopulation.empty()) {
      delete m_tPopulation.back();
      m_tPopulation.pop_back();
   }
   CRandom::RemoveCategory("ga");
   /* Other cleanup in common between master and slaves */
   Cleanup();
}

/****************************************/
/****************************************/

const CMPGA::TPopulation& CMPGA::GetPopulation() const {
   return m_tPopulation;
}

/****************************************/
/****************************************/

UInt32 CMPGA::GetGeneration() const {
   return m_unCurrentGeneration;
}

/****************************************/
/****************************************/

void CMPGA::Cleanup() {
   delete m_pcSharedMem;
}

/****************************************/
/****************************************/

void CMPGA::Evaluate() {
   /* Set parameters for the processes and resume them */
   for(UInt32 i = 0; i < m_unPopSize; ++i) {
      /* Set genome */
      m_pcSharedMem->SetGenome(i, &(m_tPopulation[i]->Genome[0]));
      /* Resume process */
      ::kill(SlavePIDs[i], SIGCONT);
   }
   /* Wait for all the slaves to finish the run */
   UInt32 unTrialsLeft = m_unPopSize;
   int nSlaveInfo;
   pid_t tSlavePID;
   while(unTrialsLeft > 0) {
      /* Wait for next slave to finish */
      tSlavePID = ::waitpid(-1, &nSlaveInfo, WUNTRACED);
      /* Make sure the slave went back to sleep and didn't crash */
      if(!WIFSTOPPED(nSlaveInfo)) {
         LOGERR << "[FATAL] Slave process with PID " << tSlavePID << " exited, can't continue. Check file ARGoS_LOGERR_" << tSlavePID << " for more information." << std::endl;
         LOG.Flush();
         LOGERR.Flush();
         Cleanup();
         ::exit(1);
      }
      /* All OK, one less slave to wait for */
      --unTrialsLeft;
   }
   /* Copy the scores into the population data */
   for(UInt32 i = 0; i < m_unPopSize; ++i) {
      m_tPopulation[i]->Score = m_pcSharedMem->GetScore(i);
   }
   /* Sort the population by score, from the best to the worst */
   std::sort(m_tPopulation.begin(),
             m_tPopulation.end(),
             m_cIndComparator);
}

/****************************************/
/****************************************/

void CMPGA::NextGen() {
   ++m_unCurrentGeneration;
   Selection();
   Crossover();
   Mutation();
}

/****************************************/
/****************************************/

bool CMPGA::Done() const {
   return m_unCurrentGeneration >= m_unGenerations;
}

/****************************************/
/****************************************/

/* Global pointer to the CMPGA object in the current slave, used by
 * SlaveHandleSIGTERM() to perform cleanup */
static CMPGA* GA_INSTANCE;

/* SIGTERM handler for slave processes */
void SlaveHandleSIGTERM(int) {
   argos::CSimulator::GetInstance().Destroy();
   argos::LOG.Flush();
   argos::LOGERR.Flush();
   GA_INSTANCE->Cleanup();
}

void CMPGA::LaunchARGoS(UInt32 un_slave_id) {
   /* Set the global GA instance pointer for signal handler */
   GA_INSTANCE = this;
   /* Install handler for SIGTERM */
   ::signal(SIGTERM, SlaveHandleSIGTERM);
   /* Initialize ARGoS */
   /* Redirect LOG and LOGERR to dedicated files to prevent clutter on the screen */
   std::ofstream cLOGFile(std::string("ARGoS_LOG_" + ToString(::getpid())).c_str(), std::ios::out);
   LOG.DisableColoredOutput();
   LOG.GetStream().rdbuf(cLOGFile.rdbuf());
   std::ofstream cLOGERRFile(std::string("ARGoS_LOGERR_" + ToString(::getpid())).c_str(), std::ios::out);
   LOGERR.DisableColoredOutput();
   LOGERR.GetStream().rdbuf(cLOGERRFile.rdbuf());
   /* The CSimulator class of ARGoS is a singleton. Therefore, to
    * manipulate an ARGoS experiment, it is enough to get its instance */
   argos::CSimulator& cSimulator = argos::CSimulator::GetInstance();
   try {
      /* Set the .argos configuration file
       * This is a relative path which assumed that you launch the executable
       * from argos3-examples (as said also in the README) */
      cSimulator.SetExperimentFileName(m_strARGoSConf);
      /* Load it to configure ARGoS */
      cSimulator.LoadExperiment();
      LOG.Flush();
      LOGERR.Flush();
   }
   catch(CARGoSException& ex) {
      LOGERR << ex.what() << std::endl;
      ::raise(SIGTERM);
   }
   /* Get a reference to the loop functions */
   CMPGALoopFunctions& cLoopFunctions = dynamic_cast<CMPGALoopFunctions&>(cSimulator.GetLoopFunctions());
   /* Create vector of scores */
   std::vector<Real> vecScores(m_unNumTrials, 0.0);
   /* Continue working until killed by parent */
   while(1) {
      /* Suspend yourself, waiting for parent's resume signal */
      ::raise(SIGTSTP);
      /* Resumed */
      /* Configure the controller with the genome */
      cLoopFunctions.ConfigureFromGenome(m_pcSharedMem->GetGenome(un_slave_id));
      /* Run the trials */
      for(size_t i = 0; i < m_unNumTrials; ++i) {
         /* Tell the loop functions to get ready for the i-th trial */
         cLoopFunctions.SetTrial(i);
         /* Reset the experiment.
          * This internally calls also CMPGALoopFunctions::Reset(). */
         cSimulator.Reset();
         /* Run the experiment */
         cSimulator.Execute();
         /* Store score */
         vecScores[i] = cLoopFunctions.Score();
         LOG.Flush();
         LOGERR.Flush();
      }
      ;
      /* Put result in shared memory */
      m_pcSharedMem->SetScore(un_slave_id, m_tScoreAggregator(vecScores));
   }
}

/****************************************/
/****************************************/

void CMPGA::Selection() {
   /* Delete all individuals apart from the top two */
   while(m_tPopulation.size() > 2) {
      delete m_tPopulation.back();
      m_tPopulation.pop_back();
   }
}

/****************************************/
/****************************************/

void CMPGA::Crossover() {
   /*
    * This is a simple one-point crossover.
    */
   SIndividual* psParent1 = m_tPopulation[0];
   SIndividual* psParent2 = m_tPopulation[1];
   UInt32 unCut;
   SIndividual* psInd;
   for(UInt32 i = 2; i < m_unPopSize; ++i) {
      /* Pick a cutting point at random */
      unCut = m_pcRNG->Uniform(CRange<UInt32>(1, m_unGenomeSize-1));
      /* Make a new individual */
      psInd = new SIndividual;
      /* Copy alleles from parent 1 */
      for(UInt32 j = 0; j < unCut; ++j) {
         psInd->Genome.push_back(psParent1->Genome[j]);
      }
      /* Copy alleles from parent 2 */
      for(UInt32 j = unCut; j < m_unGenomeSize; ++j) {
         psInd->Genome.push_back(psParent2->Genome[j]);
      }
      /* Add individual to the new population */
      m_tPopulation.push_back(psInd);
   }
}

/****************************************/
/****************************************/

void CMPGA::Mutation() {
   /* Mutate the alleles of the newly added individuals by setting a
    * new random value from a uniform distribution */
   for(UInt32 i = 2; i < m_unPopSize; ++i) {
      for(UInt32 a = 0; a < m_unGenomeSize; ++a) {
         if(m_pcRNG->Bernoulli(m_fMutationProb))
            m_tPopulation[i]->Genome[a] = m_pcRNG->Uniform(m_cAlleleRange);
      }
   }
}

/****************************************/
/****************************************/

CMPGA::CSharedMem::CSharedMem(UInt32 un_genome_size,
                              UInt32 un_pop_size) :
   m_unGenomeSize(un_genome_size),
   m_unPopSize(un_pop_size) {
   /* Create shared memory area for master-slave communication */
   m_nSharedMemFD = ::shm_open(SHARED_MEMORY_FILE.c_str(),
                               O_RDWR | O_CREAT,
                               S_IRUSR | S_IWUSR);
   if(m_nSharedMemFD < 0) {
      ::perror(SHARED_MEMORY_FILE.c_str());
      exit(1);
   }
   /* Resize shared memory area to contain the population data
    * - The area must contain m_unPopSize elements
    * - Each element must have space for the data of an individual
    *   - Genome: m_unGenomeSize * sizeof(Real)
    *   - Score: sizeof(Real)
    */
   size_t unShareMemSize = m_unPopSize * (m_unGenomeSize+1) * sizeof(Real);
   ::ftruncate(m_nSharedMemFD, unShareMemSize);
   /* Get pointer to shared memory area */
   m_pfSharedMem = reinterpret_cast<Real*>(
      ::mmap(NULL,
             unShareMemSize,
             PROT_READ | PROT_WRITE,
             MAP_SHARED,
             m_nSharedMemFD,
             0));
   if(m_pfSharedMem == MAP_FAILED) {
      ::perror("shared memory");
      exit(1);
   }
}

/****************************************/
/****************************************/

CMPGA::CSharedMem::~CSharedMem() {
   munmap(m_pfSharedMem, m_unPopSize * (m_unGenomeSize+1) * sizeof(Real));
   close(m_nSharedMemFD);
   shm_unlink(SHARED_MEMORY_FILE.c_str());
}

/****************************************/
/****************************************/


Real* CMPGA::CSharedMem::GetGenome(UInt32 un_individual) {
   return m_pfSharedMem + un_individual * (m_unGenomeSize+1);
}

/****************************************/
/****************************************/


void CMPGA::CSharedMem::SetGenome(UInt32 un_individual,
                                  const Real* pf_genome) {
   ::memcpy(m_pfSharedMem + un_individual * (m_unGenomeSize+1),
            pf_genome,
            m_unGenomeSize * sizeof(Real));
}

/****************************************/
/****************************************/

Real CMPGA::CSharedMem::GetScore(UInt32 un_individual) {
   return m_pfSharedMem[un_individual * (m_unGenomeSize+1) + m_unGenomeSize];
}

/****************************************/
/****************************************/


void CMPGA::CSharedMem::SetScore(UInt32 un_individual,
                                 Real f_score) {
   m_pfSharedMem[un_individual * (m_unGenomeSize+1) + m_unGenomeSize] = f_score;
}

/****************************************/
/****************************************/
