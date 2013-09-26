/* Include the controller definition */
#include "footbot_synchronization.h"

/****************************************/
/****************************************/

CFootBotSynchronization::CFootBotSynchronization() :
   m_pcLEDs(NULL),
   m_pcCamera(NULL),
   m_pcRNG(NULL),
   m_unCounter(0),
   m_cCountRange(0, 100) {}

/****************************************/
/****************************************/

void CFootBotSynchronization::Init(TConfigurationNode& t_node) {
   /* Get sensor/actuator handles */
   m_pcLEDs   = GetActuator<CCI_LEDsActuator                          >("leds");
   m_pcCamera = GetSensor  <CCI_ColoredBlobOmnidirectionalCameraSensor>("colored_blob_omnidirectional_camera");
   /*
    * Create a random number generator.
    * We use the 'argos' category so that creation, reset, seeding and
    * cleanup are managed by ARGoS.
    */
   m_pcRNG = CRandom::CreateRNG("argos");
   /* To make all the robots initially out of sync, choose the value of
    * the counter at random */
   m_unCounter = m_pcRNG->Uniform(m_cCountRange);
   /* Switch the camera on */
   m_pcCamera->Enable();
}

/****************************************/
/****************************************/

void CFootBotSynchronization::ControlStep() {
   /* Get led color of nearby robots */
   const CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings& sBlobs = m_pcCamera->GetReadings();
   /*
    * Check whether someone sent a 1, which means 'flash'
    */
   bool bSomeoneFlashed = false;
   for(size_t i = 0; ! bSomeoneFlashed && i < sBlobs.BlobList.size(); ++i) {
      bSomeoneFlashed = (sBlobs.BlobList[i]->Color == CColor::RED);
   }
   /*
    * If someone flashed, following Strogatz' algorithm, the counter
    * increases by an amount that depends on the value of m_unCounter.
    * For simplicity, here we just increment m_unCounter by
    * m_unCounter/10.
    * If noone flashed, then just increase the counter by 1.
    */
   if(bSomeoneFlashed) {
      m_unCounter += m_unCounter / 10;
   }
   else {
      ++m_unCounter;
   }
   /*
    * If the counter reached the max value, light up the LEDs to red and
    * zero m_unCounter.
    * Otherwise, keep the LEDs off.
   */
   if(m_unCounter > m_cCountRange.GetMax()) {
      m_pcLEDs->SetAllColors(CColor::RED);
      m_unCounter = 0;
   }
   else {
      m_pcLEDs->SetAllColors(CColor::BLACK);
   }
}

/****************************************/
/****************************************/

void CFootBotSynchronization::Reset() {
   /*
    * Reset the controller.
    * This means bringing the controller to the same state as when Init()
    * finished its execution.
    * Since we created the RNG in the 'argos' category, we don't need to
    * reset it.
    * The only thing we need to do here is resetting the counter.
    */
   m_unCounter = m_pcRNG->Uniform(m_cCountRange);
}

/****************************************/
/****************************************/

REGISTER_CONTROLLER(CFootBotSynchronization, "footbot_synchronization_controller")
