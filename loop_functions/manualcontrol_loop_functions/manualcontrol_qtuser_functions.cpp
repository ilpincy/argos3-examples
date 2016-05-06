#include "manualcontrol_qtuser_functions.h"
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <QKeyEvent>

/****************************************/
/****************************************/

static const Real DIRECTION_VECTOR_FACTOR = 10.;

/****************************************/
/****************************************/

CManualControlQTUserFunctions::CManualControlQTUserFunctions() :
   m_pcController(NULL) {
   /* No key is pressed initially */
   m_punPressedKeys[DIRECTION_FORWARD]  = 0;
   m_punPressedKeys[DIRECTION_BACKWARD] = 0;
   m_punPressedKeys[DIRECTION_LEFT]     = 0;
   m_punPressedKeys[DIRECTION_RIGHT]    = 0;
}

/****************************************/
/****************************************/

void CManualControlQTUserFunctions::KeyPressed(QKeyEvent* pc_event) {
   /* Make sure that a controller was set */
   if(!m_pcController) {
      GetQTOpenGLWidget().KeyPressed(pc_event);
      return;
   }
   switch(pc_event->key()) {
      case Qt::Key_I:
         /* Forwards */
         m_punPressedKeys[DIRECTION_FORWARD] = 1;
         SetDirectionFromKeyEvent();
         break;
      case Qt::Key_K:
         /* Backwards */
         m_punPressedKeys[DIRECTION_BACKWARD] = 1;
         SetDirectionFromKeyEvent();
         break;
      case Qt::Key_J:
         /* Left */
         m_punPressedKeys[DIRECTION_LEFT] = 1;
         SetDirectionFromKeyEvent();
         break;
      case Qt::Key_L:
         /* Right */
         m_punPressedKeys[DIRECTION_RIGHT] = 1;
         SetDirectionFromKeyEvent();
         break;
      default:
         /* Unknown key */
         GetQTOpenGLWidget().KeyPressed(pc_event);
         break;
   }
}

/****************************************/
/****************************************/

void CManualControlQTUserFunctions::KeyReleased(QKeyEvent* pc_event) {
   /* Make sure that a controller was set */
   if(!m_pcController) {
      GetQTOpenGLWidget().KeyReleased(pc_event);
      return;
   }
   switch(pc_event->key()) {
      case Qt::Key_I:
         /* Forwards */
         m_punPressedKeys[DIRECTION_FORWARD] = 0;
         SetDirectionFromKeyEvent();
         break;
      case Qt::Key_K:
         /* Backwards */
         m_punPressedKeys[DIRECTION_BACKWARD] = 0;
         SetDirectionFromKeyEvent();
         break;
      case Qt::Key_J:
         /* Left */
         m_punPressedKeys[DIRECTION_LEFT] = 0;
         SetDirectionFromKeyEvent();
         break;
      case Qt::Key_L:
         /* Right */
         m_punPressedKeys[DIRECTION_RIGHT] = 0;
         SetDirectionFromKeyEvent();
         break;
      default:
         /* Unknown key */
         GetQTOpenGLWidget().KeyReleased(pc_event);
         break;
   }
}

/****************************************/
/****************************************/

void CManualControlQTUserFunctions::EntitySelected(CEntity& c_entity) {
   /* Make sure the entity is a foot-bot */
   CFootBotEntity* pcFB = dynamic_cast<CFootBotEntity*>(&c_entity);
   if(!pcFB) return;
   /* It's a foot-bot; extract its controller */
   m_pcController = dynamic_cast<CFootBotManualControl*>(&pcFB->GetControllableEntity().GetController());
   /* Tell that foot-bot that it is selected */
   m_pcController->Select();
   /* Reset key press information */
   m_punPressedKeys[DIRECTION_FORWARD]  = 0;
   m_punPressedKeys[DIRECTION_BACKWARD] = 0;
   m_punPressedKeys[DIRECTION_LEFT]     = 0;
   m_punPressedKeys[DIRECTION_RIGHT]    = 0;
}

/****************************************/
/****************************************/

void CManualControlQTUserFunctions::EntityDeselected(CEntity& c_entity) {
   /* Make sure that a controller was set (should always be true...) */
   if(!m_pcController) return;
   /* Tell the foot-bot that it is deselected */
   m_pcController->Deselect();
   /* Forget the controller */
   m_pcController = NULL;
}

/****************************************/
/****************************************/

void CManualControlQTUserFunctions::SetDirectionFromKeyEvent() {
   /* Forward/backward direction factor (local robot X axis) */
   SInt32 FBDirection = 0;
   /* Left/right direction factor (local robot Y axis) */
   SInt32 LRDirection = 0;
   /* Calculate direction factor */
   if(m_punPressedKeys[DIRECTION_FORWARD])  ++FBDirection;
   if(m_punPressedKeys[DIRECTION_BACKWARD]) --FBDirection;
   if(m_punPressedKeys[DIRECTION_LEFT])     ++LRDirection;
   if(m_punPressedKeys[DIRECTION_RIGHT])    --LRDirection;
   /* Calculate direction */
   CVector2 cDir =
      DIRECTION_VECTOR_FACTOR *
      (CVector2(FBDirection, 0.0f) +
       CVector2(0.0f, LRDirection));
   /* Set direction */
   m_pcController->SetControlVector(cDir);
}

/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CManualControlQTUserFunctions, "manualcontrol_qtuser_functions")
