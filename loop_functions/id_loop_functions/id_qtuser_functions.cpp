#include "id_qtuser_functions.h"

/****************************************/
/****************************************/

CIDQTUserFunctions::CIDQTUserFunctions() {
   RegisterUserFunction<CIDQTUserFunctions,CFootBotEntity>(&CIDQTUserFunctions::Draw);
}

/****************************************/
/****************************************/

void CIDQTUserFunctions::Draw(CFootBotEntity& c_entity) {
   /* Disable lighting, so it does not interfere with the chosen text color */
   glDisable(GL_LIGHTING);
   /* Disable face culling to be sure the text is visible from anywhere */
   glDisable(GL_CULL_FACE);
   /* Set the text color */
   CColor cColor(CColor::BLACK);
   glColor3ub(cColor.GetRed(), cColor.GetGreen(), cColor.GetBlue());
   /* The position of the text is expressed wrt the reference point of the footbot
    * For a foot-bot, the reference point is the center of its base.
    * See also the description in
    * $ argos3 -q foot-bot
    */
   GetQTOpenGLWidget().renderText(0.0, 0.0, 0.3,             // position
                                c_entity.GetId().c_str()); // text
   /* Restore face culling */
   glEnable(GL_CULL_FACE);
   /* Restore lighting */
   glEnable(GL_LIGHTING);
}

/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CIDQTUserFunctions, "id_qtuser_functions")
