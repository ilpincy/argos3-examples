/*
 * This example shows how to define custom distributions to place the robots.
 */

#include <argos3/core/simulator/loop_functions.h>

using namespace argos;

class CCustomDistributionsLoopFunctions : public CLoopFunctions {

public:

   CCustomDistributionsLoopFunctions();
   virtual ~CCustomDistributionsLoopFunctions();

   virtual void Init(TConfigurationNode& t_tree);

private:

   /*
    *
    */
   void PlaceLine(const CVector2& c_center,
                  UInt32 un_robots,
                  Real f_distance,
                  UInt32 un_id_start);

   void PlaceCluster(const CVector2& c_center,
                     UInt32 un_robots,
                     Real f_density,
                     UInt32 un_id_start);

   void PlaceScaleFree(const CVector2& c_center,
                       UInt32 un_robots,
                       Real f_range,
                       UInt32 un_id_start);
   
private:

   enum ETopology {
      TOPOLOGY_LINE,
      TOPOLOGY_CLUSTER,
      TOPOLOGY_SCALEFREE
   };

};

