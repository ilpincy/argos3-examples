#include "id_qtuser_functions.h"

/****************************************/
/****************************************/

CIDQTUserFunctions::CIDQTUserFunctions() {
   RegisterUserFunction<CIDQTUserFunctions,CFootBotEntity>(&CIDQTUserFunctions::Draw);
}

/****************************************/
/****************************************/

void CIDQTUserFunctions::Draw(CFootBotEntity& c_entity) {
   /* The position of the text is expressed wrt the reference point of the footbot
    * For a foot-bot, the reference point is the center of its base.
    * See also the description in
    * $ argos3 -q foot-bot
    */
   DrawText(CVector3(0.0, 0.0, 0.3),   // position
            c_entity.GetId().c_str()); // text
}

/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CIDQTUserFunctions, "id_qtuser_functions")
