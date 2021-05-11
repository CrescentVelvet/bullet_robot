#include "OpenSpeed.h"
#include "CommandFactory.h"
#include "DribbleStatus.h"
#include "VisionModule.h"
#include "robot_power.h"
#include "parammanager.h"
#include "kickregulation.h"
#include "KickStatus.h"
#include <cmath>
#include <fstream>
#include <iostream>

namespace  {
bool SAVE_DATA = false;
std::ofstream file;
const double maxAcc = 3000;
double SHOOT_ACCURACY;
}
COpenSpeed::COpenSpeed() {
    ZSS::ZParamManager::instance()->loadParam(SAVE_DATA, "ZAutoFit/TestSpeedConstant", false);
    ZSS::ZParamManager::instance()->loadParam(SHOOT_ACCURACY, "GetBall/ShootAccuracy", 5.0);
}

CPlayerCommand* COpenSpeed::execute(const CVisionModule* pVision) {
	int num = task().executor;
	double speedX = task().player.speed_x; // x方向平动速度
	double speedY = task().player.speed_y; // y方向平动速度
	double speedR = task().player.rotate_speed; // 转动速度
    double dribblePower = 0;
    CVector localVelocity = VisionModule::Instance()->previousLocalCommand(num, 1);
    CVector commandVelocity = pVision->ourPlayer(num).Vel();
    CVector Acc = CVector(speedX, speedY) - localVelocity;
    if(Acc.mod() >= maxAcc/PARAM::Vision::FRAME_RATE) Acc = Acc.unit() *  maxAcc/PARAM::Vision::FRAME_RATE;
    else if(SAVE_DATA&&sqrt(speedX*speedX+speedY*speedY)>100&&localVelocity.mod()/sqrt(speedX*speedX+speedY*speedY)>0.75){
        std::ofstream file ("data/Speedconstant.txt",std::ios::app);
        if(file.is_open()){
            file<<" "<<num<<" "<<sqrt(speedX*speedX+speedY*speedY)<<" "<<localVelocity.mod()<<" "<<std::endl;
            file.close();
        }
        else {
            qDebug() << "file write error";
        }
    }
    CVector localcommand = localVelocity + Acc;
    speedX = localcommand.x();
    speedY = localcommand.y();
    if(task().player.flag & PlayerStatus::KICK){
        CGeoPoint targetPos = task().player.pos;
        double power = task().player.kickpower;
        KickStatus::Instance()->setKick(num, power);
    }
    if(task().player.flag & PlayerStatus::DRIBBLING || task().player.flag & PlayerStatus::DRIBBLE){
        dribblePower = 3;
    }
    return CmdFactory::Instance()->newCommand(CPlayerSpeedV2(num, speedX, speedY, speedR, task().player.use_dir, dribblePower));
}

