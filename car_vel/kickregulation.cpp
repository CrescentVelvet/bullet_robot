#include "kickregulation.h"
#include "staticparams.h"
#include "Global.h"
#include "GDebugEngine.h"
#include "parammanager.h"

namespace  {
double ROLL_FRICTION_RADIO[2] = {1.05, 1.28};
double HIGH_REGULATION_RADIO = 3.75;
double DRIBBLE_POWER_RADIO = 1;
double FLAT_KICK_MAX = 6300.0;
bool IS_SIMULATION = false;
bool KICK_DEBUG = true;
bool IS_RIGHT = false;
bool DEBUG_PRINT = false;
const double DRIBBLE_CHIP_DIR = 50.0 * PARAM::Math::PI / 180.0;
}

CKickRegulation::CKickRegulation()
{
    ZSS::ZParamManager::instance()->loadParam(IS_SIMULATION,"Alert/IsSimulation",false);
    ZSS::ZParamManager::instance()->loadParam(KICK_DEBUG, "KickRegulation/Debug", true);
    ZSS::ZParamManager::instance()->loadParam(DEBUG_PRINT, "Debug/KickRegulation", false);
    ZSS::ZParamManager::instance()->loadParam(ROLL_FRICTION_RADIO[0], "KickRegulation/lowPowerRadio", 1.05);
    ZSS::ZParamManager::instance()->loadParam(ROLL_FRICTION_RADIO[1], "KickRegulation/HighPowerRadio", 1.28);
    ZSS::ZParamManager::instance()->loadParam(HIGH_REGULATION_RADIO, "KickRegulation/HighRadio4Vx/Vy", 3.75);
    ZSS::ZParamManager::instance()->loadParam(DRIBBLE_POWER_RADIO, "KickRegulation/dribblePowerRadio", 1.0);
    ZSS::ZParamManager::instance()->loadParam(FLAT_KICK_MAX, "KickLimit/FlatKickMax", 6300.0);
    ZSS::ZParamManager::instance()->loadParam(IS_RIGHT, "ZAlert/IsRight", false);
}

bool CKickRegulation::regulate(int player, const CGeoPoint& target, double& needBallVel, double& playerDir, bool isChip) {
    auto& ball = vision->ball();
    auto& kicker = vision->ourPlayer(player);

    if (std::fabs(kicker.Dir() - playerDir) > PARAM::Math::PI / 2 ) {
        return false;
    }

    double originVel = needBallVel;
    double vel2targetDir;
    double tanVel2Target;
    double paralVel2Target;
    double ballRotVel;
//    if (ball.Valid()) {
//        vel2targetDir = Utils::Normalize(ball.RawVel().dir() - playerDir);
//        tanVel2Target = ball.RawVel().mod() * std::sin(vel2targetDir);
//        paralVel2Target = ball.RawVel().mod() * std::cos(vel2targetDir);
//        ballRotVel = 0;
//    } else {
        vel2targetDir = Utils::Normalize(kicker.RawVel().dir() - playerDir);
        tanVel2Target = kicker.RawVel().mod() * std::sin(vel2targetDir);//????????????????????????????????????
        paralVel2Target = kicker.RawVel().mod() * std::cos(vel2targetDir);//????????????????????????????????????
        ballRotVel = kicker.ImuRotateVel() * PARAM::Vehicle::V2::PLAYER_CENTER_TO_BALL_CENTER;//??????????????????????????????????????????
//    }


    CVector originDirVector = CVector(needBallVel,0).rotate(playerDir);
    GDebugEngine::Instance()->gui_debug_line(kicker.Pos(), kicker.Pos() + originDirVector, COLOR_WHITE);
    //????????????
    //1. ????????????
    double tanVelCompensate = (tanVel2Target + ballRotVel);// * std::cos(vel2targetDir));
    tanVelCompensate *= std::abs(kicker.ImuRotateVel()) / 1.5;
    //todo!!!! tanvel dir is wrong!!!
    //paralVel2Target -= ballRotVel * std::sin(vel2targetDir);
    if(false){
        GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(0, 0), QString("kicker_vel: %1  ballrot_vel: %2").arg(kicker.RawVel().mod()).arg(ballRotVel).toLatin1());
        GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(0, -200), QString("tan_vel: %1  paral_vel: %2").arg(tanVelCompensate).arg(paralVel2Target).toLatin1());
    }
//    if (isChip && !IS_SIMULATION) {
//        tanVelCompensate *= 1;
//    }

    //2. ??????????????????
    if (IS_SIMULATION) {
        paralVel2Target = 0;
    } else {
        if (isChip) {
            needBallVel = std::sqrt(9800.0 * needBallVel/ (2 * std::tan(DRIBBLE_CHIP_DIR)));//??????????????????????????????x????????????
            needBallVel = (-paralVel2Target + std::sqrt(paralVel2Target * paralVel2Target + 2 * originVel * 9800.0 / std::tan(DRIBBLE_CHIP_DIR))) / 2;//??????????????????vy = vx*tan50???????????????t???????????????(originVel)??????????????????vx????????????????????????????????????????????????????????????
        }
        else {//some trick
            needBallVel -= paralVel2Target;
//            if (needBallVel / std::fabs(tanVelCompensate) < HIGH_REGULATION_RADIO) {
//                needBallVel *= ROLL_FRICTION_RADIO[1];
//            } else {
//                needBallVel *= ROLL_FRICTION_RADIO[0];
//            }
        }
    }

    //3. ?????????????????????
    double compensateDir = std::atan2(tanVelCompensate, needBallVel);
    playerDir -= compensateDir;

    //4. ?????????????????????
    needBallVel = std::sqrt(std::pow(needBallVel, 2) + std::pow(tanVelCompensate, 2));
    if (!(IS_SIMULATION || isChip)) {
        needBallVel *= DRIBBLE_POWER_RADIO;
    }
    if (isChip) {
        needBallVel = 2 * std::pow(needBallVel, 2) * std::tan(DRIBBLE_CHIP_DIR) / 9800.0;
    }

//    needBallVel = needBallVel > 6000 ? 6000 : needBallVel;
//    //?????????????????????
//    if (Utils::InTheirPenaltyArea(target, 0)) {
//        needBallVel = std::max(6500.0, needBallVel);
//    }

    if (KICK_DEBUG) {
        GDebugEngine::Instance()->gui_debug_line(kicker.Pos(), kicker.Pos() + CVector(5000,0).rotate(kicker.Dir()));
        GDebugEngine::Instance()->gui_debug_line(kicker.Pos(), kicker.Pos() + CVector(5000,0).rotate(kicker.Dir() + 3 * PARAM::Math::PI / 180), COLOR_GRAY);
        GDebugEngine::Instance()->gui_debug_line(kicker.Pos(), kicker.Pos() + CVector(5000,0).rotate(kicker.Dir() - 3 * PARAM::Math::PI / 180), COLOR_GRAY);
        GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(-5000, -3500) * (IS_RIGHT ? -1 : 1), QString("vy: %1  vx: %2  rotV: %3  v: %4").arg(paralVel2Target).arg(tanVel2Target).arg(kicker.RotVel()).arg(vel2targetDir).toLatin1(),COLOR_ORANGE);
        GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(-5000, -3300) * (IS_RIGHT ? -1 : 1), QString("oV: %1   V: %2").arg(originVel).arg(needBallVel).toLatin1(),COLOR_ORANGE);
        CVector dirVector = CVector(needBallVel,0).rotate(playerDir);
        GDebugEngine::Instance()->gui_debug_line(kicker.Pos(), kicker.Pos() + dirVector, COLOR_YELLOW);//?????????????????????
        CVector tanVel2TargetVector = CVector(tanVelCompensate,0).rotate(originDirVector.dir()-PARAM::Math::PI/2.0);// (target - kicker.Pos()).rotate(-PARAM::Math::PI/2.0) / (target - kicker.Pos()).mod() * tanVelCompensate;
        CVector paralVel2TargetVector = CVector(paralVel2Target,0).rotate(originDirVector.dir());// (target - kicker.Pos())/ (target - kicker.Pos()).mod() * paralVel2Target;
        GDebugEngine::Instance()->gui_debug_line(kicker.Pos(), kicker.Pos() + tanVel2TargetVector, COLOR_PURPLE);//????????????
        GDebugEngine::Instance()->gui_debug_line(kicker.Pos(), kicker.Pos() + paralVel2TargetVector, COLOR_GREEN);//??????????????????
        GDebugEngine::Instance()->gui_debug_line(kicker.Pos(), kicker.Pos() + kicker.RawVel(), COLOR_RED);
    }

    return true;
}

double CKickRegulation::regulateCheck(int player, double needBallVel, double needDir, bool isChip, double tolerance) {
    double param_ball = 1.35; // ??????
    auto& kicker = vision->ourPlayer(player);
    double vel2targetDir, needTanVel, needParalVel;
    double vel2faceDir, tanVel, paralVel, temp_tanVel, temp_ballRotVel, now_ballRotVel;//???????????????
    double vel2targetDir_img, needTanVel_img, needParalVel_img, paralVel_img;
    double imu_rovel = kicker.ImuRotateVel() * PARAM::Vehicle::V2::PLAYER_CENTER_TO_BALL_CENTER;
    double real_kickerDir = Utils::Normalize(kicker.ImuDir());
    double kickerDir = Utils::Normalize(kicker.ImuDir());
//    if (std::abs(imu_rovel) < 530.0) {
//        kickerDir = real_kickerDir;
//    }
//    else if (std::abs(imu_rovel) < 700.0) {
//        kickerDir = real_kickerDir + imu_rovel * 1 / 6000;
//    }
//    else {
//        kickerDir = real_kickerDir + imu_rovel * 2 / 6000;
//    }
    if(isChip) {
        needBallVel = std::sqrt(9800.0 * needBallVel / (2 * std::tan(DRIBBLE_CHIP_DIR)));
    }
    if(!IS_SIMULATION) {
        vel2targetDir = Utils::Normalize(needDir) - kickerDir;
        vel2targetDir_img = Utils::Normalize(needDir) - Utils::Normalize(kicker.Dir());
    }
    else {
        vel2targetDir = Utils::Normalize(needDir) - Utils::Normalize(kicker.Dir());
        vel2targetDir_img = Utils::Normalize(needDir) - Utils::Normalize(kicker.Dir());
    }
    needTanVel = needBallVel * std::sin(vel2targetDir);//????????????
    needParalVel = needBallVel * std::cos(vel2targetDir);//????????????
    needTanVel_img = needBallVel * std::sin(vel2targetDir_img);
    needParalVel_img = needBallVel * std::cos(vel2targetDir_img);
    float vel2faceDir_img;
    float tanVel_img, temp_tanVel_img;
    if(!IS_SIMULATION){
        vel2faceDir = Utils::Normalize(kicker.RawVel().dir()) - kickerDir;
        vel2faceDir_img = Utils::Normalize(kicker.RawVel().dir()) - Utils::Normalize(kicker.Dir());
    }
    else {
        vel2faceDir = vel2faceDir_img = Utils::Normalize(kicker.RawVel().dir()) - Utils::Normalize(kicker.Dir());
    }
    paralVel = kicker.RawVel().mod() * std::cos(vel2faceDir);
    paralVel_img = kicker.RawVel().mod() * std::cos(vel2faceDir_img);
    if(!IS_SIMULATION) {
        temp_ballRotVel = kicker.ImuRotateVel() * PARAM::Vehicle::V2::PLAYER_CENTER_TO_BALL_CENTER;
        now_ballRotVel = kicker.ImuRotateVel() * PARAM::Vehicle::V2::PLAYER_CENTER_TO_BALL_CENTER;
    }
    else {
        temp_ballRotVel = kicker.RotVel() * PARAM::Vehicle::V2::PLAYER_CENTER_TO_BALL_CENTER;
        now_ballRotVel = kicker.RotVel() * PARAM::Vehicle::V2::PLAYER_CENTER_TO_BALL_CENTER;
    }
    if(!IS_SIMULATION) {
        now_ballRotVel *= std::abs(kicker.ImuRotateVel()) / param_ball;
    }
    temp_tanVel = (0.1 * kicker.RawVel().mod() * std::sin(vel2faceDir) + temp_ballRotVel) * temp_ballRotVel / 800;
    temp_tanVel_img = kicker.RawVel().mod() * std::sin(vel2faceDir_img) + temp_ballRotVel;
    tanVel = kicker.RawVel().mod() * std::sin(vel2faceDir) + now_ballRotVel;
    tanVel_img = kicker.RawVel().mod() * std::sin(vel2faceDir_img) + now_ballRotVel;
    double now_rot2vell = std::asin(tanVel / needBallVel);
    double act_rot2vell = std::asin(temp_tanVel / needBallVel);
    double big_rot2vell = 0.42 * std::atan2(temp_tanVel * temp_tanVel, std::sqrt(needBallVel*needBallVel-temp_tanVel*temp_tanVel+temp_tanVel*temp_tanVel*temp_tanVel*temp_tanVel));
//    double act_ball2car = std::asin(needTanVel / needBallVel);
    double act_ball2car = vel2targetDir;
    double fit_ball2car = big_rot2vell * imu_rovel / std::abs(imu_rovel);
    double old_fit = 19.190577857534688*act_rot2vell*act_rot2vell*act_rot2vell+-0.2924188081034752*act_rot2vell*act_rot2vell+-0.8329685836993624*act_rot2vell;
//    double fit_ball2car = 0.27734032759846383*big_rot2vell*big_rot2vell*big_rot2vell+-0.043449314014431195*big_rot2vell*big_rot2vell;
//    double fit_ball2car = param*(19.190577857534688*act_rot2vell*act_rot2vell*act_rot2vell-0.2924188081034752*act_rot2vell*act_rot2vell+0.3329685836993624*act_rot2vell);
//    double fit_ball2car = param*(30.190577857534688*act_rot2vell*act_rot2vell*act_rot2vell+-0.2924188081034752*act_rot2vell*act_rot2vell+-0.8329685836993624*act_rot2vell);
//    double fit_ball2car = 0.42*(19.190577857534688*big_rot2vell*big_rot2vell*big_rot2vell+-0.2924188081034752*big_rot2vell*big_rot2vell+-0.8329685836993624*big_rot2vell);
    double error_temp = std::abs(Utils::Normalize(act_ball2car - big_rot2vell));
    double error_imu = std::abs(Utils::Normalize(act_ball2car - now_rot2vell));
    double error_temp_img = std::abs(Utils::Normalize(std::asin(needTanVel_img / needBallVel) - std::asin(temp_tanVel_img / needBallVel)));
    double error_img = std::abs(Utils::Normalize(std::asin(needTanVel_img / needBallVel) - std::asin(tanVel_img / needBallVel)));
    if (std::abs(imu_rovel) > 500.0) {
        tolerance = tolerance * std::abs(imu_rovel) / 1500;
    }
//    if (imu_rovel > 800) {
//        tolerance = 5 * tolerance;
//    }
    if(DEBUG_PRINT){
//        GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(0,1400), QString("real_kickerDir: %1 kickerDir: %2").arg(real_kickerDir).arg(kickerDir).toLatin1(),COLOR_GREEN);
//        GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(0,1200), QString("old_fit: %1 temp_ballRotVel: %2").arg(old_fit).arg(temp_ballRotVel).toLatin1(),COLOR_GREEN);
//        GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(0,1000), QString("KickerVel: %1 temp_ballRotVel: %2").arg(kicker.RawVel().mod() * std::sin(vel2faceDir)).arg(temp_ballRotVel).toLatin1(),COLOR_GREEN);
//        GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(0,800), QString("needDir: %1 kickerDir: %2 kickerDir_img: %3").arg(needDir).arg(kickerDir).arg(kicker.Dir()).toLatin1(),COLOR_GREEN);
//        GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(0,600), QString("temp_tanVel: %1 needBallVel: %2").arg(temp_tanVel).arg(needBallVel).toLatin1(),COLOR_GREEN);
//        GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(0,400), QString("error_temp: %1 error_imu: %2").arg(error_temp).arg(error_imu).toLatin1(),COLOR_GREEN);
//        GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(0,200), QString("big_rot2vell: %1 error_imu: %2").arg(big_rot2vell).arg(error_imu).toLatin1(),COLOR_GREEN);
//        GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(0,0), QString("act_ball2car: %1 now_rot2vell: %2").arg(act_ball2car).arg(now_rot2vell).toLatin1(),COLOR_GREEN);
//        GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(0,-200), QString("fit_ball2car: %1 act_rot2vell: %2").arg(fit_ball2car).arg(act_rot2vell).toLatin1(),COLOR_GREEN);
        GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(0,-400), QString("imu_rovel: %1 tolerance: %2").arg(imu_rovel).arg(tolerance).toLatin1(),COLOR_GREEN);
//        GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(0,-600), QString("error_temp_img: %1 error_img: %2").arg(error_temp_img).arg(error_img).toLatin1(),COLOR_GREEN);
//        GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(0,-800), QString("needDir: %1  kickerDir: %2 velDir: %3").arg(needDir).arg(kickerDir).arg(kicker.RawVel().dir()).toLatin1(),COLOR_GREEN);
//        GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(0,-1000), QString("needvp: %1  needvt: %2 2targetdir: %3").arg(needParalVel).arg(needTanVel).arg(vel2targetDir).toLatin1(),COLOR_GREEN);
//        GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(0,-1200), QString("realvp: %1   realvt: %2 2veldir:%3").arg(paralVel).arg(tanVel).arg(vel2faceDir).toLatin1(),COLOR_GREEN);
        GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(0,-1400), QString("ballrot: %1 error_imu: %2 error_img: %3").arg(now_ballRotVel).arg(error_imu).arg(error_img).toLatin1(),COLOR_GREEN);
        GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(0,-1600), QString("ImuRotateVel: %1 ImuDir: %2 RawRotVel: %3 RotVel: %4 Dir: %5").arg(kicker.ImuRotateVel()).arg(Utils::Normalize(kicker.ImuDir())).arg(kicker.RawRotVel()).arg(kicker.RotVel()).arg(kicker.Dir()).toLatin1(),COLOR_GREEN);
    }
    double tolerance_img = 2 * tolerance;
    if(error_imu > tolerance || (imu_rovel > 100.0 && act_ball2car * imu_rovel < 0)) {
        return 0.0;
    }
    if(isChip){
        if(IS_SIMULATION) {
            return (2 * std::pow(needBallVel, 2) * std::tan(DRIBBLE_CHIP_DIR) / 9800.0);
        }
        needBallVel = (-paralVel + std::sqrt(paralVel * paralVel + 2 * needParalVel * 9800.0 / std::tan(DRIBBLE_CHIP_DIR))) / 2;
        return (2 * std::pow(needBallVel, 2) * std::tan(DRIBBLE_CHIP_DIR) / 9800.0);
    }
    if(IS_SIMULATION) {
        return std::sqrt(needBallVel * needBallVel - temp_tanVel * temp_tanVel);
    }
    if(needBallVel < 1100) {
        return std::sqrt(needBallVel * needBallVel - temp_tanVel * temp_tanVel);
    }
    return std::sqrt(needBallVel * needBallVel - temp_tanVel * temp_tanVel) - paralVel;//needParalVel - paralVel;
}
