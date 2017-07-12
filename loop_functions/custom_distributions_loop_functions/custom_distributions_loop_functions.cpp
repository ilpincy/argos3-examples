#include "custom_distributions_loop_functions.h"

#include <argos3/plugins/simulator/entities/box_entity.h>
#include <argos3/plugins/simulator/entities/cylinder_entity.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>

#include <sstream>
#include <list>

/****************************************/
/****************************************/

static const Real        FB_RADIUS        = 0.085036758f;
static const Real        FB_AREA          = ARGOS_PI * Square(0.085036758f);
static const std::string FB_CONTROLLER    = "ffc";
static const UInt32      MAX_PLACE_TRIALS = 20;
static const UInt32      MAX_ROBOT_TRIALS = 20;

/****************************************/
/****************************************/

CCustomDistributionsLoopFunctions::CCustomDistributionsLoopFunctions() {
}

/****************************************/
/****************************************/

CCustomDistributionsLoopFunctions::~CCustomDistributionsLoopFunctions() {
}

/****************************************/
/****************************************/

void CCustomDistributionsLoopFunctions::Init(TConfigurationNode& t_tree) {
   try {
      /*
       * Parse the configuration file
       */
      UInt32 unPlacedRobots = 0;
      /* Go through the nodes */
      TConfigurationNodeIterator itDistr;
      for(itDistr = itDistr.begin(&t_tree);
          itDistr != itDistr.end();
          ++itDistr) {
         /* Make sure a known distribution was passed */
         if(itDistr->Value() != "line" &&
            itDistr->Value() != "cluster" &&
            itDistr->Value() != "scalefree") {
            THROW_ARGOSEXCEPTION("Unknown topology \"" << itDistr->Value() << "\"");
         }
         /* Get current node */
         TConfigurationNode& tDistr = *itDistr;
         /* Parse common attributes */
         /* Distribution center */
         CVector2 cCenter;
         GetNodeAttribute(tDistr, "center", cCenter);
         /* Number of robots to place */
         UInt32 unRobots;
         GetNodeAttribute(tDistr, "robot_num", unRobots);
         /* Parse distribution-specific attributes and place robots */
         if(itDistr->Value() == "line") {
            /* Distance between the robots */
            Real fDistance;
            GetNodeAttribute(tDistr, "robot_distance", fDistance);
            /* Place robots */
            PlaceLine(cCenter, unRobots, fDistance, unPlacedRobots);
         }
         else if(itDistr->Value() == "cluster") {
            /* Density of the robots */
            Real fDensity;
            GetNodeAttribute(tDistr, "robot_density", fDensity);
            /* Place robots */
            PlaceCluster(cCenter, unRobots, fDensity, unPlacedRobots);
         }
         else /* (itDistr->Value() == "scalefree") */ {
            /* Range around each robot */
            Real fRange;
            GetNodeAttribute(tDistr, "robot_range", fRange);
            /* Place robots */
            PlaceScaleFree(cCenter, unRobots, fRange, unPlacedRobots);
         }
         /* Update robot count */
         unPlacedRobots += unRobots;
      }
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing the loop functions", ex);
   }
}

/****************************************/
/****************************************/

void CCustomDistributionsLoopFunctions::PlaceLine(const CVector2& c_center,
                                                  UInt32 un_robots,
                                                  Real f_distance,
                                                  UInt32 un_id_start) {
   try {
      CFootBotEntity* pcFB;
      std::ostringstream cFBId;
      /* For each robot */
      for(size_t i = 0; i < un_robots; ++i) {
         /* Make the id */
         cFBId.str("");
         cFBId << "fb" << (i + un_id_start);
         /* Create the robot in the origin and add it to ARGoS space */
         pcFB = new CFootBotEntity(
            cFBId.str(),
            FB_CONTROLLER,
            CVector3(i + c_center.GetX(), i + c_center.GetY(), 0));
         AddEntity(*pcFB);
      }
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("While placing robots in a line", ex);
   }
}

/****************************************/
/****************************************/

void CCustomDistributionsLoopFunctions::PlaceCluster(const CVector2& c_center,
                                                     UInt32 un_robots,
                                                     Real f_density,
                                                     UInt32 un_id_start) {
   try {
      /* Calculate side of the region in which the robots are scattered */
      Real fHalfSide = Sqrt((FB_AREA * un_robots) / f_density) / 2.0f;
      CRange<Real> cAreaRange(-fHalfSide, fHalfSide);
      /* Place robots */
      UInt32 unTrials;
      CFootBotEntity* pcFB;
      std::ostringstream cFBId;
      CVector3 cFBPos;
      CQuaternion cFBRot;
      /* Create a RNG (it is automatically disposed of by ARGoS) */
      CRandom::CRNG* pcRNG = CRandom::CreateRNG("argos");
      /* For each robot */
      for(size_t i = 0; i < un_robots; ++i) {
         /* Make the id */
         cFBId.str("");
         cFBId << "fb" << (i + un_id_start);
         /* Create the robot in the origin and add it to ARGoS space */
         pcFB = new CFootBotEntity(
            cFBId.str(),
            FB_CONTROLLER);
         AddEntity(*pcFB);
         /* Try to place it in the arena */
         unTrials = 0;
         bool bDone;
         do {
            /* Choose a random position */
            ++unTrials;
            cFBPos.Set(pcRNG->Uniform(cAreaRange) + c_center.GetX(),
                       pcRNG->Uniform(cAreaRange) + c_center.GetY(),
                       0.0f);
            cFBRot.FromAngleAxis(pcRNG->Uniform(CRadians::UNSIGNED_RANGE),
                                 CVector3::Z);
            bDone = MoveEntity(pcFB->GetEmbodiedEntity(), cFBPos, cFBRot);
         } while(!bDone && unTrials <= MAX_PLACE_TRIALS);
         if(!bDone) {
            THROW_ARGOSEXCEPTION("Can't place " << cFBId.str());
         }
      }
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("While placing robots in a cluster", ex);
   }
}

/****************************************/
/****************************************/

struct SFData {
   
   struct SEntry {
      UInt32 Conns;
      CVector3& Pos;
      SEntry(UInt32 un_conns,
             CVector3& c_pos) :
         Conns(un_conns),
         Pos(c_pos) {}
   };

   SFData() :
      TotConns(0),
      RNG(CRandom::CreateRNG("argos")) {}

   ~SFData() {
      while(!Data.empty()) {
         delete Data.front();
         Data.pop_front();
      }
   }

   void Insert(CFootBotEntity& c_entity) {
      /* Two connections to be added: entity <-> pivot */
      TotConns += 2;
      Data.push_back(
         new SEntry(1,
                    c_entity.GetEmbodiedEntity().
                    GetOriginAnchor().Position));
   }

   SEntry* Pick() {
      if(Data.size() > 1) {
         /* More than 1 element stored, look for the pivot */
         UInt32 x = RNG->Uniform(CRange<UInt32>(0, TotConns));
         UInt32 unSum = 0;
         std::list<SEntry*>::iterator it = Data.begin();
         while(it != Data.end() && unSum <= x) {
            unSum += (*it)->Conns;
            ++it;
         }
         if(it != Data.end()) {
            --it;
            return *it;
         }
         else {
            return Data.back();
         }
      }
      else if(Data.size() == 1) {
         /* One element stored, just return that one */
         return Data.front();
      }
      else THROW_ARGOSEXCEPTION("SFData::Pick(): empty structure");
   }

private:

   std::list<SEntry*> Data;
   UInt32 TotConns;
   CRandom::CRNG* RNG;
   
};

static Real GenerateCoordinate(CRandom::CRNG* pc_rng,
                               Real f_half_range) {
   Real v = pc_rng->Uniform(CRange<Real>(-f_half_range, f_half_range));
   if(v > 0.0) v += f_half_range;
   else v -= f_half_range;
   return v;
}

void CCustomDistributionsLoopFunctions::PlaceScaleFree(const CVector2& c_center,
                                                       UInt32 un_robots,
                                                       Real f_range,
                                                       UInt32 un_id_start) {
   try {
      /* Data structures for the insertion of new robots */
      UInt32 unRobotTrials, unPlaceTrials, unPivot;
      CFootBotEntity* pcFB;
      std::ostringstream cFBId;
      CVector3 cFBPos;
      CQuaternion cFBRot;
      SFData sData;
      SFData::SEntry* psPivot;
      bool bDone;
      Real fHalfRange = f_range * 0.5;
      /* Create a RNG (it is automatically disposed of by ARGoS) */
      CRandom::CRNG* pcRNG = CRandom::CreateRNG("argos");
      /* Add first robot in the origin */
      /* Create the robot in the origin and add it to ARGoS space */
      cFBId << "fb" << un_id_start;
      pcFB = new CFootBotEntity(
         cFBId.str(),
         FB_CONTROLLER);
      AddEntity(*pcFB);
      MoveEntity(pcFB->GetEmbodiedEntity(),
                 CVector3(c_center.GetX(),
                          c_center.GetY(),
                          0.0),
                 CQuaternion());
      sData.Insert(*pcFB);
      /* Add other robots */
      for(UInt32 i = 1; i < un_robots; ++i) {
         /* Make the id */
         cFBId.str("");
         cFBId << "fb" << (i + un_id_start);
         /* Create the robot in the origin and add it to ARGoS space */
         pcFB = new CFootBotEntity(
            cFBId.str(),
            FB_CONTROLLER);
         AddEntity(*pcFB);
         /* Retry choosing a pivot until you get a position or have an error */
         unRobotTrials = 0;
         do {
            /* Choose a pivot */
            ++unRobotTrials;
            psPivot = sData.Pick();
            cFBRot.FromAngleAxis(pcRNG->Uniform(CRadians::UNSIGNED_RANGE),
                                 CVector3::Z);
            /* Try placing a robot close to this pivot */
            unPlaceTrials = 0;
            do {
               ++unPlaceTrials;
               /* Pick a position within the range of the pivot */
               cFBPos.Set(GenerateCoordinate(pcRNG, fHalfRange) + c_center.GetX(),
                          GenerateCoordinate(pcRNG, fHalfRange) + c_center.GetY(),
                          0.0f);
               cFBPos += psPivot->Pos;
               /* Try placing the robot */
               bDone = MoveEntity(pcFB->GetEmbodiedEntity(), cFBPos, cFBRot);
            }
            while(!bDone && unPlaceTrials <= MAX_PLACE_TRIALS);
         } while(!bDone && unRobotTrials <= MAX_ROBOT_TRIALS);
         /* Was the robot placed successfully? */
         if(!bDone) {
            THROW_ARGOSEXCEPTION("Can't place " << cFBId.str());
         }
         /* Yes, insert it in the data structure */
         ++psPivot->Conns;
         sData.Insert(*pcFB);
      }
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("While placing robots in a scale-free distribution", ex);
   }
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CCustomDistributionsLoopFunctions, "custom_distributions_loop_functions");
