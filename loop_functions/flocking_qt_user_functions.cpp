#include "flocking_qt_user_functions.h"
#include <controllers/pathfindbot-controller.h>
#include <argos3/core/simulator/entity/controllable_entity.h>

using namespace argos;

/****************************************/
/****************************************/

CFlockingQTUserFunctions::CFlockingQTUserFunctions()/*:
  loopFunctions(dynamic_cast<CFlockingLoopFunctions&>(CSimulator::GetInstance().GetLoopFunctions()))
 */{
   RegisterUserFunction<CFlockingQTUserFunctions,CFootBotEntity>(&CFlockingQTUserFunctions::Draw);
   RegisterUserFunction<CFlockingQTUserFunctions, CFootBotEntity>(&CFlockingQTUserFunctions::DrawOnRobot);
}

/****************************************/
/****************************************/

void CFlockingQTUserFunctions::Draw(CFootBotEntity& c_entity) {
   //CFootBotFlocking& cController = dynamic_cast<CFootBotFlocking&>(c_entity.GetControllableEntity().GetController());
  
}

void CFlockingQTUserFunctions::DrawOnRobot(CFootBotEntity& entity) {
	CPathFindBot& c = dynamic_cast<CPathFindBot&>(entity.GetControllableEntity().GetController());
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
	
	// Disable for now
	//GetOpenGLWidget().renderText(0.0, 0.0, 0.5,             // position
	//			     entity.GetId().c_str()); // text
	
		DrawText(CVector3(0.0, 0.0, 0.3),   // position
    entity.GetId().c_str()); // text
	/* Restore face culling */
	glEnable(GL_CULL_FACE);
	/* Restore lighting */
	glEnable(GL_LIGHTING);
}


/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CFlockingQTUserFunctions, "flocking_qt_user_functions")
