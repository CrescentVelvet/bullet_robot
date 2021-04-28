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
        double vel = getVel();
        recordVelData(i, false, power, send_power, vel);
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
        double dist = getDist(i);
        recordVelData(i, false, power, send_power, dist);
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

double CAutoShootFit::getDist(int id) {
    double dist = -1.0;
//    double vel = GlobalData::instance()->maintain[0].ball->velocity.mod();
//    std::ofstream ratio_file("/home/zjunlict-vision-1/Desktop/dhz/Kun2/ZBin/data/ChipBallVel.txt", std::ios::app);
//    if(chip_oldvel == -1.0) {
//        chip_oldvel = vel;
//    }
//    if(chip_oldvel < vel) {
//        chip_oldvel - vel;
//    }
//    else {
//        qDebug() << "chip~~~~~~";
//    }
//    if(chip_oldvel - vel > 2) {
//        chip_oldvel = vel;
//        CGeoPoint chip_pos = GlobalData::instance()->maintain[0].ball[0].pos;
//        dist = chip_pos.dist(chip_robot_pos);
//        qDebug() << "loading data---";
//    }
//    qDebug() << "getdist" << vel;
//    if(ratio_file.is_open()) {
//        ratio_file << " " << id << " " << vel << " " << chip_oldvel << " " << dist << std::endl;
//        ratio_file.close();
//    }

    bool isChip = GlobalData::instance()->maintain[0].ball[0].ball_state_machine == 10;
    static bool lastStatus = false;
//    qDebug() << GlobalData::instance()->maintain[0].robot[1][id].pos.x() << GlobalData::instance()->maintain[0].robot[1][id].pos.y(); //先读取踢球数据再更新踢球车的坐标
    if (isChip) { lastStatus = true; }
    if (lastStatus && !isChip) {
        CGeoPoint chip_pos = GlobalData::instance()->maintain[0].ball[0].pos;
        double dist = chip_pos.dist(chip_robot_pos);
//        qDebug() << GlobalData::instance()->maintain[0].robot[1][id].pos.x() << GlobalData::instance()->maintain[0].robot[1][id].pos.y();
//        if ( abs(chip_pos.x()) < PARAM::Field::PITCH_LENGTH/2 && abs(chip_pos.y()) < PARAM::Field::PITCH_WIDTH/2) {
        if ( GlobalData::instance()->maintain[0].robot[1][id].pos.x() > 0 && GlobalData::instance()->maintain[0].robot[1][id].pos.y() > 0) {
            lastStatus = false;
            GDebugEngine::instance()->gui_debug_msg(CGeoPoint(-1000,0),QString("getDist: %1").arg(dist).toLatin1(), COLOR_RED);
            qDebug() << "loading data---" << dist;
            return dist;
        }
    }
    return -1.0;
}

double CAutoShootFit::getVel() {
    double maxv = GlobalData::instance()->maintain[0].ball->velocity.mod();
    return maxv;
}

void CAutoShootFit::recordVelData(int id, bool mode, int my_power, double sp, double vel_or_dist) {
    bool is_FlatChip = false;
    char *file_name;
    double my_vel = vel_or_dist;
    if (is_FlatChip){
        file_name = "/home/zjunlict-vision-1/Desktop/dhz/Kun2/ZBin/data/VelData.txt";
    }
    else {
        file_name = "/home/zjunlict-vision-1/Desktop/dhz/Kun2/ZBin/data/ChipData.txt";
    }
    std::ofstream ratio_file(file_name, std::ios::app);
    if (my_maxvel < my_vel) {
        my_maxvel = my_vel;
    }
    if (my_oldid != id) {
        my_oldid = id;
        my_maxvel = -1;
    }
    if(ratio_file.is_open() && my_power != -1) {
        ratio_file << " " << id << " " << my_vel << " " << my_maxvel << " " << my_power << std::endl;
        ratio_file.close();
    }
//    qDebug() << "id " << id << "my_vel " << my_vel << "my_maxvel " << my_maxvel << "my_power " << my_power;
}
