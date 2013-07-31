#ifndef FORAGING_LOOP_FUNCTIONS_H
#define FORAGING_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/simulator/entity/floor_entity.h>
#include <argos3/core/utility/math/range.h>
#include <argos3/core/utility/math/rng.h>

using namespace argos;

class CForagingLoopFunctions : public CLoopFunctions {

public:

   CForagingLoopFunctions();
   virtual ~CForagingLoopFunctions() {}

   virtual void Init(TConfigurationNode& t_tree);
   virtual void Reset();
   virtual void Destroy();
   virtual CColor GetFloorColor(const CVector2& c_position_on_plane);
   virtual void PreStep();

private:

   Real m_fFoodSquareRadius;
   CRange<Real> m_cForagingArenaSideX, m_cForagingArenaSideY;
   std::vector<CVector2> m_cFoodPos;
   CFloorEntity* m_pcFloor;
   CRandom::CRNG* m_pcRNG;

   std::string m_strOutput;
   std::ofstream m_cOutput;

   UInt32 m_unCollectedFood;
   SInt64 m_nEnergy;
   UInt32 m_unEnergyPerFoodItem;
   UInt32 m_unEnergyPerWalkingRobot;
};

#endif
