#ifndef FLOCKING_QT_USER_FUNCTIONS_H
#define FLOCKING_QT_USER_FUNCTIONS_H

#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
//#include <loop_functions/flocking_loop_functions/flocking_loop_functions.h>

using namespace argos;

class CFlockingQTUserFunctions : public CQTOpenGLUserFunctions {

public:

   CFlockingQTUserFunctions();

   virtual ~CFlockingQTUserFunctions() {}

   void Draw(CFootBotEntity& c_entity);
   void DrawOnRobot(argos::CFootBotEntity& entity);
   
   //CFlockingLoopFunctions& loopFunctions;
   
};

#endif
