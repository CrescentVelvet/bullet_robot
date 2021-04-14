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
    std::thread* receiveThread;


    //udpsend param
    bool send = false;
    bool sendAll = false;
    int team;
    int id;


    //test mode
    double send_power;

    //kick param
    const int max_record_num = 5;
    bool flatparamtest;
    bool chipparamtest;
    bool isWaiting = true;
    bool isGettingVel = false;
    bool isGettingDist = false;
    bool isChanged[PARAM::ROBOTNUM*PARAM::TEAMS] = {false};
    bool calculated[PARAM::ROBOTNUM] = {false};
    double vel[PARAM::ROBOTNUM][max_record_num];
    double dist[PARAM::ROBOTNUM][max_record_num];
    int p[PARAM::ROBOTNUM][max_record_num];
    int cnt[PARAM::Field::MAX_PLAYER]={0};
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
//    my_oldpower = -1000;
    my_maxvel = -1000;
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
        double dist = getDist();
        recordVelData(i, false, power, send_power, dist);
        if (dist > 0 ) {
            reset();
        }
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

double CAutoShootFit::getDist() {
    bool isChip =  GlobalData::instance()->maintain[0].ball[0].ball_state_machine == 10;
    static bool lastStatus = false;
    if (isChip) {
        lastStatus = true;
    }
    if (lastStatus && !isChip) {
        CGeoPoint chip_pos = GlobalData::instance()->maintain[0].ball[0].pos;
        double dist = chip_pos.dist(chip_robot_pos);
        if (abs(chip_pos.x()) < PARAM::Field::PITCH_LENGTH/2 && abs(chip_pos.y()) < PARAM::Field::PITCH_WIDTH/2 ) {
            lastStatus = false;
            GDebugEngine::instance()->gui_debug_msg(CGeoPoint(-1000,0),QString("%1").arg(dist).toLatin1());
            return dist;
        }
    }
    return -1.0;
}

double CAutoShootFit::getVel() {
    double maxv = -1.0;
    double v = GlobalData::instance()->maintain[0].ball->velocity.mod();
    if (v > max_vel[2] and v < max_vel[1]) {
        max_vel[2] = v;
    }
    else if (v > max_vel[1] and v < max_vel[0]) {
        max_vel[2] = max_vel[1];
        max_vel[1] = v;
    }
    else if (v > max_vel[0]) {
        max_vel[2] = max_vel[1];
        max_vel[1] = max_vel[0];
        max_vel[0] = v;
    }
    cycle_cnt ++;
    if(cycle_cnt == 30) {
        maxv= (max_vel[0] + max_vel[1] + max_vel[2]) / 3;
        if(maxv>0){

        }
        else {
            qDebug() << "error when getvel";
        }
        for (int i=0; i<3; i++) {
            max_vel[i] = -1;
        }
    }
    return maxv;
}

void CAutoShootFit::recordVelData(int id, bool mode, int my_power, double sp,double vel_or_dist) {
    std::ofstream ratio_file("/home/zjunlict-vision-1/Desktop/dhz/Kun2/ZBin/data/VelData.txt", std::ios::app);
    double my_vel = GlobalData::instance()->maintain[0].ball->velocity.mod();
    if (my_maxvel < my_vel){
        my_maxvel = my_vel;
    }
    if (my_oldid != id){
        my_oldid = id;
        my_maxvel = -1000;
    }
    if(ratio_file.is_open() && my_power != -1){
        ratio_file << " " << id << " " << my_vel << " " << my_maxvel << " " << my_power << std::endl;
        ratio_file.close();
    }
    qDebug() << "id " << id << "my_vel " << my_vel << "my_maxvel " << my_maxvel << "my_power " << my_power;
}
