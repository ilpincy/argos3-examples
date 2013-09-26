/*
 * AUTHOR: Carlo Pinciroli <cpinciro@ulb.ac.be>
 *
 * An example synchronization controller for the foot-bot based on the
 * synchronized oscillators of Mirollo and Strogatz. See paper at:
 * http://www.math.pitt.edu/~bard/classes/mth3380/syncpapers/Mirollo-Strogatz.pdf
 *
 * The experiment reproduces the synchronization mechanisms presented by
 * Mirollo and Strogatz in their paper.
 * The robots initially flash their LEDs out of synchrony. Using the
 * camera, they can perceive if other robots around have flashed their
 * LEDs. Read the paper for more information about the details.
 *
 * This controller is meant to be used with the configuration file:
 *    experiments/synchronization.argos
 */

#ifndef FOOTBOT_SYNCHRONIZATION_H
#define FOOTBOT_SYNCHRONIZATION_H

/*
 * Include some necessary headers.
 */
/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
/* Definition of the LEDs actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>
/* Definition of the camera sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_colored_blob_omnidirectional_camera_sensor.h>
/* Definitions for random number generation */
#include <argos3/core/utility/math/rng.h>

using namespace argos;

class CFootBotSynchronization : public CCI_Controller {

public:

   CFootBotSynchronization();
   virtual ~CFootBotSynchronization() {}

   virtual void Init(TConfigurationNode& t_node);
   virtual void ControlStep();
   virtual void Reset();
   virtual void Destroy() {}

private:

   /* Pointer to the LEDs actuator */
   CCI_LEDsActuator* m_pcLEDs;
   /* Pointer to the omnidirectional camera sensor */
   CCI_ColoredBlobOmnidirectionalCameraSensor* m_pcCamera;

   /* The random number generator */
   CRandom::CRNG* m_pcRNG;

   /* An internal counter. When the counter reaches 10, the robot
      flashes. */
   UInt32 m_unCounter;
   /* The counter range */
   CRange<UInt32> m_cCountRange;
};

#endif
