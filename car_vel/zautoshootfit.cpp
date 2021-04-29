#include "zautoshootfit.h"
#include "visionmodule.h"
#include "globaldata.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <thread>
#include "parammanager.h"
#include <QIODevice>
#include <QFile>
#include "zss_kickparam.pb.h"
#include "staticparams.h"
#include <thread>
#include "chipsolver.h"
#include "globaldata.h"
#include "maintain.h"
#include "debugengine.h"
#include <fstream>

namespace  {
    auto zpm = ZSS::ZParamManager::instance();
    //udpsend param
    bool send = false;
    bool sendAll = false;
    int team;
    int id;
    //test mode
    double send_power;
    //kick param
    bool flatparamtest;
    bool chipparamtest;
    bool isWaiting = true;
    bool isGettingVel = false;
    bool isGettingDist = false;
    double max_vel[3];
    int cycle_cnt = 0;
    int power = -1;
    int kicker_team;
    bool mode;
    int i;
    CGeoPoint chip_robot_pos;
}

CAutoShootFit::CAutoShootFit()
{
    zpm->loadParam(_isrun,"ZAutoFit/isrun",false);
    zpm->loadParam(flatparamtest, "ZAutoFit/flatparamtest", false);
    zpm->loadParam(chipparamtest, "ZAutoFit/chipparamtest", false);
    zpm->loadParam(send, "ZAutoFit/send", false);
    zpm->loadParam(sendAll, "ZAutoFit/sendAll", false);
    zpm->loadParam(team, "ZAutoFit/team", 0);
    zpm->loadParam(id, "ZAutoFit/id", 0);
    chip_oldvel = -1.0;
    my_maxvel = -1.0;
    my_oldid = -1;
    sendSocket.setSocketOption(QAbstractSocket::MulticastTtlOption, 1);
}

CAutoShootFit::~CAutoShootFit() {
    sendSocket.disconnectFromHost();
    receiveSocket.disconnectFromHost();
}

void CAutoShootFit::run() {
    if (flatparamtest && !chipparamtest) {
        GDebugEngine::instance()->gui_debug_msg(CGeoPoint(-2000,3000),QString("FlatPARAMTEST is running").toLatin1(),COLOR_GREEN);
        flatParamTest();
    }
    else if (chipparamtest && !flatparamtest) {
        GDebugEngine::instance()->gui_debug_msg(CGeoPoint(-2000,3000),QString("ChipPARAMTEST is running").toLatin1(),COLOR_GREEN);
        chipParamTest();
    }
    else if (flatparamtest && chipparamtest) {
        GDebugEngine::instance()->gui_debug_msg(CGeoPoint(-2000,3000),QString("Flip222Chip").toLatin1(),COLOR_RED);
    }
}

void CAutoShootFit::reset() {
    isWaiting = true;
    isGettingVel = false;
    isGettingDist = false;
    power = -1;
    cycle_cnt = 0;
    kicker_team = -1;
    i = -1;
}

void CAutoShootFit::flatParamTest() {
    if(!isWaiting && GlobalData::instance()->robotInformation[kicker_team][i].flat) {
        isGettingVel = true;
        isWaiting = false;
    }
    if (isGettingVel) {
        double vel = recordFlatData(i, power, send_power);
        if (vel > 0) { reset(); }
    }
}

void CAutoShootFit::chipParamTest() {
    if (!isWaiting && GlobalData::instance()->robotInformation[kicker_team][i].chip) {
        isGettingDist = true;
        const OriginMessage &robot_vision = GlobalData::instance()->processRobot[0];
        chip_robot_pos = robot_vision.robot[kicker_team][i].pos;
    }
    if (isGettingDist) {
        double dist = recordChipData(i, power, send_power);
        if (dist > 0 ) { reset(); }
    }
}

void CAutoShootFit::getKickerMes(int t, int n, bool m) {
    kicker_team = t;
    i = n;
    mode = m;
    isWaiting = false;
}

void CAutoShootFit::getKickPower(int p, double sp) {
    power = p;
    send_power = sp;
}

double CAutoShootFit::recordFlatData(int id, int my_power, double send_power) {
    std::ofstream ratio_file("/home/zjunlict-vision-1/Desktop/dhz/Kun2/ZBin/data/FlatData.txt", std::ios::app);
    double my_vel = GlobalData::instance()->maintain[0].ball->velocity.mod();
    if (my_maxvel < my_vel) {
        my_maxvel = my_vel;
    }
    if (my_oldid != id) {
        my_oldid = id;
        my_maxvel = -1.0;
    }
    if(ratio_file.is_open() && my_power != -1) {
        ratio_file << " " << id << " " << my_vel << " " << my_maxvel << " " << my_power << " " << send_power << std::endl;
        ratio_file.close();
    }
    return my_vel;
}

double CAutoShootFit::recordChipData(int id, int my_power, double send_power) {
    CGeoPoint chip_pos = GlobalData::instance()->maintain[0].ball[0].pos;
    double my_dist = chip_pos.dist(chip_robot_pos);
    double ball_vel = GlobalData::instance()->maintain[0].ball[0].velocity.mod();
    std::ofstream ratio_file("/home/zjunlict-vision-1/Desktop/dhz/Kun2/ZBin/data/ChipData.txt", std::ios::app);
    if(ratio_file.is_open() && my_power != -1) {
        ratio_file << " " << id << " " << ball_vel << " " << my_dist << " " << my_power << " " << send_power << std::endl;
        ratio_file.close();
    }
    return my_dist;
}
