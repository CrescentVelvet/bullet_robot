#include "actionmodule.h"
#include "parammanager.h"
#include "globaldata.h"
#include "dealrobot.h"
#include "messageinfo.h"
#include <QDateTime>
#include <QtDebug>
#include <chrono>
#include <thread>
#include <cmath>
#include <fstream>
#include "staticparams.h"
#include "networkinterfaces.h"
#include "robotsensor.h"
#include "zautoshootfit.h"
#include "debugengine.h"
#include <QSettings>

namespace ZSS {
namespace {
const int TRANSMIT_PACKET_SIZE = 25;const int TRANS_FEEDBACK_SIZE = 26;
const int TRANSMIT_START_PACKET_SIZE = 6;
const int PORT_SEND = 1030;
const int PORT_RECEIVE = 1030;
const double FULL_BATTERY = 224.0;
const double LOW_BATTERY = 196.0;
const double FULL_CAPACITANCE = 254.0;
const double LOW_CAPACITANCE = 29.0;
auto zpm = ZSS::ZParamManager::instance();
bool usedir;
quint8 kickStandardization(int, quint8, bool, quint16, double);
const QStringList radioSendAddress2choose=  {"10.12.225.142", "10.12.225.143", "10.12.225.130","10.12.225.109","10.12.225.78"};
const QStringList radioReceiveAddress2choose =  {"10.12.225.142", "10.12.225.143", "10.12.225.130","10.12.225.110","10.12.225.79"};
QString radioSendAddress[PARAM::TEAMS] = {"10.12.225.142","10.12.225.130"};
QString radioReceiveAddress[PARAM::TEAMS] = { "10.12.225.142","10.12.225.130"};
int blue_sender_interface,blue_receiver_interface,yellow_sender_interface,yellow_receiver_interface;
std::thread* receiveThread = nullptr;
bool IS_SIMULATION;
double VIEW_FREQUENCE;
double imu_dir;
double imu_rotate_vel;
bool NEED_SAVE_DATA;
bool NEED_SAVE_COMMAND;
double Normalize(double angle);
template<typename T>
T limitRange(const T& value,const T& minValue,const T& maxValue) {
    return value > maxValue ? maxValue : (value < minValue) ? minValue : value;
}
}

ActionModule::ActionModule(QObject *parent) : QObject(parent), team{-1, -1} {
//    qDebug() << "ActionModule------";
    blue_sender_interface = blue_receiver_interface = yellow_sender_interface = yellow_receiver_interface = 0;
    tx.resize(TRANSMIT_PACKET_SIZE);
    tx[0] = 0x40;
    if(receiveSocket.bind(QHostAddress::AnyIPv4, PORT_RECEIVE, QUdpSocket::ShareAddress | QUdpSocket::ReuseAddressHint)) {
        qDebug() << "****** start receive ! ******";
        receiveThread = new std::thread([ = ] {readData();});
        receiveThread->detach();
    } else {
        qDebug() << "Bind Error in action module !";
    }
    for (int j = 0; j < PARAM::TEAMS; j++){
        reportVec[j] = -1;
        for (int i = 0; i < PARAM::ROBOTNUM; i++) {
            no_response[j][i] = false;
        }
    }
    zpm->loadParam(VIEW_FREQUENCE, "Debug/VIEW_FREQUENCE", 1.0);
    zpm->loadParam(usedir, "IMU/useDir", true);
    ZSS::ZParamManager::instance()->loadParam(NEED_SAVE_DATA, "Alert/SaveRecMsg", false);
    ZSS::ZParamManager::instance()->loadParam(NEED_SAVE_COMMAND, "Alert/SaveCommand", false);
}

ActionModule::~ActionModule() {
    sendSocket.disconnectFromHost();
    receiveSocket.disconnectFromHost();
}

bool ActionModule::connectRadio(int id, int frq) {
//    qDebug() << "connectRadio------";
    bool color;
    if(id >= 0 && id < PARAM::TEAMS) {
        zpm->loadParam(color, "ZAlert/IsYellow", false);
        team[id] = color ? PARAM::YELLOW : PARAM::BLUE;
        qDebug() << "connectRadio : " << id << (color ? "YELLOW" : "BLUE") << frq;
        sendStartPacket(id, frq);
        return true;
    } else
        qDebug() << "ERROR id in connectRadio function!";
    return false;
}

bool ActionModule::disconnectRadio(int id) {
    if(id >= 0 && id < PARAM::TEAMS) {
        team[id] = -1;
        sendSocket.disconnectFromHost();
        return true;
    } else
        qDebug() << "ERROR id in disconnectRadio function!";
    return false;
}
void ActionModule::setSimulation(bool simulation){
//    qDebug() << "setSimulation------";
    IS_SIMULATION = simulation;
}
void ActionModule::sendStartPacket(int t, int frequency) {
//    qDebug() << "sendStartPacket------";
    // this 't' is id
    QByteArray startPacketSend(TRANSMIT_START_PACKET_SIZE, 0);
    QByteArray startPacketReceive(TRANSMIT_START_PACKET_SIZE, 0);
    if(frequency == 8) {
        startPacketSend[0] = (char)0xf0;
        startPacketSend[1] = (char)0x5a;
        startPacketSend[2] = (char)0x5a;
        startPacketSend[3] = (char)0x01;
        startPacketSend[4] = (char)0x02;
        startPacketSend[5] = (char)0xa7;

        startPacketReceive[0] = (char)0xf0;
        startPacketReceive[1] = (char)0x5a;
        startPacketReceive[2] = (char)0x5a;
        startPacketReceive[3] = (char)0x02;
        startPacketReceive[4] = (char)0x02;
        startPacketReceive[5] = (char)0xa8;
    } else if(frequency == 6) {
        startPacketSend[0] = (char)0xf0;
        startPacketSend[1] = (char)0x18;
        startPacketSend[2] = (char)0x5a;
        startPacketSend[3] = (char)0x01;
        startPacketSend[4] = (char)0x02;
        startPacketSend[5] = (char)0x65;

        startPacketReceive[0] = (char)0xf0;
        startPacketReceive[1] = (char)0x18;
        startPacketReceive[2] = (char)0x18;
        startPacketReceive[3] = (char)0x02;
        startPacketReceive[4] = (char)0x02;
        startPacketReceive[5] = (char)0x24;
    } else if(frequency == 1) {
        startPacketSend[0] = (char)0xf0;
        startPacketSend[1] = (char)0x58;
        startPacketSend[2] = (char)0x58;
        startPacketSend[3] = (char)0x01;
        startPacketSend[4] = (char)0x02;
        startPacketSend[5] = (char)0xa3;

        startPacketReceive[0] = (char)0xf0;
        startPacketReceive[1] = (char)0x58;
        startPacketReceive[2] = (char)0x58;
        startPacketReceive[3] = (char)0x02;
        startPacketReceive[4] = (char)0x02;
        startPacketReceive[5] = (char)0xa4;
    } else if(frequency == 2) {
        startPacketSend[0] = (char)0xf0;
        startPacketSend[1] = (char)0x64;
        startPacketSend[2] = (char)0x64;
        startPacketSend[3] = (char)0x01;
        startPacketSend[4] = (char)0x02;
        startPacketSend[5] = (char)0xbb;

        startPacketReceive[0] = (char)0xf0;
        startPacketReceive[1] = (char)0x64;
        startPacketReceive[2] = (char)0x64;
        startPacketReceive[3] = (char)0x02;
        startPacketReceive[4] = (char)0x02;
        startPacketReceive[5] = (char)0xbc;
    }   else if(frequency == 11) {
        startPacketSend[0] = (char)0xf0;
        startPacketSend[1] = (char)0x66;
        startPacketSend[2] = (char)0x66;
        startPacketSend[3] = (char)0x01;
        startPacketSend[4] = (char)0x02;
        startPacketSend[5] = (char)0xbf;

        startPacketReceive[0] = (char)0xf0;
        startPacketReceive[1] = (char)0x66;
        startPacketReceive[2] = (char)0x66;
        startPacketReceive[3] = (char)0x02;
        startPacketReceive[4] = (char)0x02;
        startPacketReceive[5] = (char)0xc0;
    }
        else{
        qDebug() << "Frequency ERROR !!!";
    }
//    sendSocket.setMulticastInterface(ZNetworkInterfaces::instance()->getFromIndex(0));
//    receiveSocket.setMulticastInterface(ZNetworkInterfaces::instance()->getFromIndex(0));
    qDebug() << "start package" << startPacketSend;
    sendSocket.writeDatagram(startPacketSend, TRANSMIT_START_PACKET_SIZE, QHostAddress(radioSendAddress[t]), PORT_SEND);
    receiveSocket.writeDatagram(startPacketReceive, TRANSMIT_START_PACKET_SIZE, QHostAddress(radioReceiveAddress[t]), PORT_SEND);
    std::this_thread::sleep_for(std::chrono::milliseconds(4));
    qDebug() << "Frequency:" << frequency << " Send IP:" << radioSendAddress[t] << " Receive IP:" << radioReceiveAddress[t];
}

void ActionModule::sendLegacy(int t, const ZSS::Protocol::Robots_Command& commands) {
//    qDebug() << "sendLegacy------";
    // this 't' is color
    auto& socket = sendSocket;
    int id = -1;
    if(t == team[0])
        id = 0;
    else if(t == team[1])
        id = 1;
    else
        return;
    reportVec[id] = -1;
    int size = commands.command_size();
    int count = 0;
    tx.fill(0x00);
    tx[0] = 0x40;
//    qDebug() << size;
    for(int i = 0; i < size; i++) {
        if(count == 4) {
            socket.writeDatagram(tx.data(), TRANSMIT_PACKET_SIZE, QHostAddress(radioSendAddress[id]), PORT_SEND);
            std::this_thread::sleep_for(std::chrono::microseconds(3000));
            tx.fill(0x00);
            tx[0] = 0x40;
            count = 0;
        }
        auto& command = commands.command(i);
        encodeLegacy(command, this->tx, count++, id);
    }
    socket.writeDatagram(tx.data(), TRANSMIT_PACKET_SIZE, QHostAddress(radioSendAddress[id]), PORT_SEND);
    if (true) {
        QDateTime utc_time = QDateTime::fromTime_t( QDateTime::currentDateTimeUtc().toTime_t() );
        QString current_time = utc_time .toString("yyyy.MM.dd hh:mm:ss");
        GDebugEngine::instance()->gui_debug_msg(CGeoPoint(-4800,-3150),QString("time : %1").arg(current_time).toLatin1(),COLOR_GREEN);
        for(int i = 0; i < PARAM::ROBOTNUM; i++) {
            double id_length = 400 * i;
            GDebugEngine::instance()->gui_debug_msg(CGeoPoint(-4800+id_length,-3300),QString("id:%1").arg(i).toLatin1(),COLOR_GREEN);
            GDebugEngine::instance()->gui_debug_msg(CGeoPoint(-4800+id_length,-3450),QString("b:%1").arg(kick_param[i].fb).toLatin1(),COLOR_GREEN);
            GDebugEngine::instance()->gui_debug_msg(CGeoPoint(-4800+id_length,-3600),QString("c:%1").arg(kick_param[i].fc).toLatin1(),COLOR_GREEN);
        }
    }
}

void ActionModule::readData() {
//    qDebug() << "readData------";
    static QHostAddress address;
    static int color;
    while(true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        if(!IS_SIMULATION) {
            for (int color = PARAM::BLUE; color <= PARAM::YELLOW; color++) {
                for (int j = 0; j < PARAM::ROBOTNUM; j++ ) {
                    QDateTime UTC(QDateTime::currentDateTimeUtc());
                    qint64 current_time = UTC.toMSecsSinceEpoch();
                    if (current_time - last_receive_time[color][j] > 3 * VIEW_FREQUENCE * 1000 && !no_response[j]) {
                        no_response[color][j] = true;
                        GlobalData::instance()->robotInfoMutex.lock();
                        GlobalData::instance()->robotInformation[color][j].infrared = false;
                        GlobalData::instance()->robotInformation[color][j].flat = false;
                        GlobalData::instance()->robotInformation[color][j].chip = false;
                        //GlobalData::instance()->robotInformation[color][j].battery = 0;
                        GlobalData::instance()->robotInfoMutex.unlock();
                        emit receiveRobotInfo(color, j);
                    }
                }
            }
        }
        while (receiveSocket.state() == QUdpSocket::BoundState && receiveSocket.hasPendingDatagrams()) {
//            qDebug() << "receive data !!!";
//            auto msgInfo = (MessageInfo*)(MessageInfo::instance());
//            char newInfo = msgInfo->info() | 0x02;
            //        msgInfo->setInfo(newInfo);
            rx.resize(receiveSocket.pendingDatagramSize());
            receiveSocket.readDatagram(rx.data(), rx.size(), &address);
            color = (address.toString() == radioReceiveAddress[0]) ? team[0] : team[1];
            if (color == -1) {
//                qDebug() << "Receive Error Message from:" << address << "in actionmodule.cpp";
                break;
            }
            auto& data = rx;
            int id = 0;
            bool infrared = false;
            bool flat = false;
            bool chip = false;
            int battery = 0;
            int capacitance = 0;
            short wheelVel[4] = {0};
            int truepower = 0;

            if(data[0] == (char)0xff && data[1] == (char)0x02) {
                id       = (quint8)data[2];
                infrared = (quint8)data[3] & 0x40;
                flat     = (quint8)data[3] & 0x20;
                chip     = (quint8)data[3] & 0x10;
                battery  = (quint8)data[4];
                capacitance = (quint8)data[5];
                wheelVel[0] = (quint8)(data[6])*256 + (quint8)data[7];
                wheelVel[0] = wheelVel[0]>32767 ? wheelVel[0]-65535:wheelVel[0];
                wheelVel[1] = (quint8)(data[8])*256 + (quint8)data[9];
                wheelVel[1] = wheelVel[1]>32767 ? wheelVel[1]-65535:wheelVel[1];
                wheelVel[2] = (quint8)(data[10])*256 + (quint8)data[11];
                wheelVel[2] = wheelVel[2]>32767 ? wheelVel[2]-65535:wheelVel[2];
                wheelVel[3] = (quint8)(data[12])*256 + (quint8)data[13];
                wheelVel[3] = wheelVel[3]>32767 ? wheelVel[3]-65535:wheelVel[3];
                imu_dir = ((quint8)(data[14]) + 256 * (qint8)(data[15]));
                imu_rotate_vel = ((quint8)(data[16]) + 256 * (qint8)(data[17]));
                //truepower = (quint16)data[12];
                imu_dir = imu_dir > 32767 ? imu_dir - 65535 : imu_dir;
                imu_rotate_vel = imu_rotate_vel > 32767 ? imu_rotate_vel - 65535 : imu_rotate_vel;
                imu_dir = imu_dir / 100.0 * ZSS::Sim::PI / 180.0;
                imu_rotate_vel = imu_rotate_vel / 100.0 * ZSS::Sim::PI / 180.0 * 2;
                //qDebug()<<"imu_dir"<<imu_dir<<color<<id;
                GlobalData::instance()->robotInfoMutex.lock();
                GlobalData::instance()->robotInformation[color][id].rawImuDir = imu_dir;
                if(abs(wheelVel[0])<10 && abs(wheelVel[1])<10 && abs(wheelVel[2])<10 && abs(wheelVel[3])<10 && GlobalData::instance()->maintain[-1].robot[color][id].rotateVel<1){
//                    if(id == 15) qDebug()<<"image dir: "<<GlobalData::instance()->processRobot[0].robot[color][id].angle;
                    RobotSensor::instance()->imuZero[color][id] = imu_dir - GlobalData::instance()->processRobot[0].robot[color][id].angle;
                    RobotSensor::instance()->imuCorrect[color][id]++;
                    GlobalData::instance()->robotInformation[color][id].needreport = false;
                    GlobalData::instance()->robotInformation[color][id].imucleaned = true;
                    GlobalData::instance()->robotInformation[color][id].imuavailable = true;
//                    if(id == 15) qDebug()<<"imu clean"; //keke
                }
//                if(id == 15) qDebug()<<"lun xun: "<<id<<" "<< imu_dir; //keke
                if(reportVec[color] == id ){
                    reportVec[color] = -1;
                    GlobalData::instance()->robotInformation[color][id].needreport = false;
                }
//                    qDebug() << color << id << "raw imu dir: "<<imu_dir<<"imu rot_vel"<<imu_rotate_vel<<"bias: "<<GlobalData::instance()->imuZero[color][id]<<"battery"<<battery;

//                else if(fabs(imu_rotate_vel) < 0.02 && GlobalData::instance()->cleaned[color]){
//                    GlobalData::instance()->checkimu(color, id, side);
//                }
                GlobalData::instance()->robotInformation[color][id].imuDir = imu_dir - RobotSensor::instance()->imuZero[color][id];
                GlobalData::instance()->robotInformation[color][id].imuRotateDir = imu_rotate_vel;
                //qDebug() << color << id << "raw imu dir: "<<imu_dir<<"imu rot_vel"<<imu_rotate_vel<<"bias: "<<GlobalData::instance()->imuZero[color][id]<<"battery"<<battery;
                GlobalData::instance()->robotInformation[color][id].infrared = infrared;
                GlobalData::instance()->robotInformation[color][id].flat = flat;
                GlobalData::instance()->robotInformation[color][id].chip = chip;
                GlobalData::instance()->robotInformation[color][id].battery = battery/256.0;//std::min(std::max((battery - LOW_BATTERY) / (FULL_BATTERY - LOW_BATTERY), 0.0), 1.0);
                GlobalData::instance()->robotInformation[color][id].capacitance = std::min(std::max((capacitance - LOW_CAPACITANCE) / (FULL_CAPACITANCE - LOW_CAPACITANCE), 0.0), 1.0);
                GlobalData::instance()->robotInfoMutex.unlock();
                emit receiveRobotInfo(color, id);
            }
            QDateTime UTC(QDateTime::currentDateTimeUtc());
            last_receive_time[color][id] = UTC.toMSecsSinceEpoch();
            no_response[color][id] = false;
            if(NEED_SAVE_DATA){
                saveData(color, id, last_receive_time[color][id], wheelVel);
            }
            //qDebug() << rx.toHex();
        }
    }
}



QStringList ActionModule::getAllAddress(){
//    qDebug() << "getAllAddress------";
    return radioSendAddress2choose;
}


QString ActionModule::getRealAddress(int index){
    return radioSendAddress[index];
}


void ActionModule::changeAddress(int team, int index){
    radioSendAddress[team] = radioSendAddress2choose[index];
    radioReceiveAddress[team] = radioReceiveAddress2choose[index];
}

void ActionModule::saveData(int team, int id, qint64 time, short *wheelVel){
    std::ofstream file ("Receivedata",std::ios::app);
    if(file.is_open()){
        file<<" "<<id<<" "<<" "<<time<<" "<<wheelVel[0]<<" "<<wheelVel[1]<<" "<<wheelVel[2]<<" "<<wheelVel[3]<<" "<<std::endl;
        file.close();
    }
    else {
        qDebug() << "file write error";
    }
}

void ActionModule::saveCommand(int id, qint64 time, double vx, double vy, double vr){
    std::ofstream file ("SendCommands",std::ios::app);
    if(file.is_open()){
        file << id << " " << time << " " << vx <<" "<< vy << " " << vr << " " << std::endl;
        file.close();
    }
    else {
        qDebug() << "file write error";
    }
}

void ActionModule::encodeLegacy(const ZSS::Protocol::Robot_Command& command, QByteArray& tx, int num, int team) {
//    const static qint16 V_MAX = (1 << 9) - 1;
//    const static qint16 VR_MAX = (1 << 11) - 1;
    // send back to vision module
    // num 0 ~ 3
    // id  0 ~ 15
    quint8 id = (quint8)command.robot_id();
    double origin_vx = command.velocity_x() / 10.0; //mm -> cm
    double origin_vy = command.velocity_y() / 10.0; //mm -> cm
    bool use_dir = command.use_dir();
    double origin_vr = command.velocity_r();
    double vel_hope = command.raw_power();
//    double dt = 1. / Athena::FRAME_RATE;
//    double theta = origin_vr * dt;
//    CVector v(origin_vx, origin_vy);
//    v = v.rotate(theta);

    QDateTime UTC(QDateTime::currentDateTimeUtc());
    qint64 current_time = UTC.toMSecsSinceEpoch();

    bool report = usedir? GlobalData::instance()->robotInformation[team][id].needreport : false;
    report = report || (((current_time - ZActionModule::instance()->last_receive_time[team][id]) >= 3 * VIEW_FREQUENCE * 1000) && (reportVec[team] < 0));
//    if(id == 5) report = true;
    if (report) {
        reportVec[team] = id;
    }
//    if (fabs(theta) > 0.00001) {
//        //            if (i==0) cout << theta << " " <<vx << " "<< vy << " ";
//        v = v * theta / (2 * sin(theta / 2));
//        origin_vx = v.x();
//        origin_vy = v.y();
//        //            if (i==0) cout << vx << " "<< vy << " " << endl;
//    }
    origin_vr = use_dir ? Normalize(origin_vr + RobotSensor::instance()->imuZero[team][id]) : origin_vr;
    qint16 vx = (qint16)(origin_vx);
    qint16 vy = (qint16)(origin_vy);
    qint16 vr = (qint16)(origin_vr * /*(command.use_dir()?650:*/160/*)*/);
//    if(use_dir && id ==15) qDebug()<<"use dir: "<<id<<"angle: "<<vr;
//    vx = limitRange(vx,(qint16)-V_MAX,V_MAX);
//    vy = limitRange(vy,(qint16)-V_MAX,V_MAX);
//    vr = limitRange(vr,(qint16)-VR_MAX,VR_MAX);
    if(NEED_SAVE_COMMAND) {
        saveCommand(id, current_time, origin_vx, origin_vy, origin_vr);
    }
    qint16 abs_vx = std::abs(vx);
    qint16 abs_vy = std::abs(vy);
    qint16 abs_vr = std::abs(vr);
    // flat&chip m/s -> cm/s
    // kick   1 : chip   0 : flat
    bool kick = command.kick();
    quint16 speed = command.power(); //mm
    quint8 power = 0;
    double send_power;
    if(speed > 5) {
        send_power = speed;
        if (AutoShootFit::instance()->getRun()) {
            power = (quint8)speed;
        } else {
            power = kickStandardization(team, id, kick, speed, vel_hope);
        }
        AutoShootFit::instance()->getKickPower(power, send_power);
    }
//     dribble -1 ~ +1 -> -3 ~ +3
    qint8 dribble = command.dribbler_spin() > 0.5 ? 3 : 0;
    tx[0] = (tx[0]) | (1 << (3 - num));
    tx[num * 4 + 1] = ((quint8)report << 7) | ((quint8)kick << 6) | dribble << 4 | id;
    tx[num * 4 + 2] = (vx >> 8 & 0x80) | (abs_vx & 0x7f);
    tx[num * 4 + 3] = (vy >> 8 & 0x80) | (abs_vy & 0x7f);
    tx[num * 4 + 4] = (vr >> 8 & 0x80) | (abs_vr & 0x7f);
    tx[num  + 17] = (abs_vx >> 1 & 0xc0) | (abs_vy >> 3 & 0x30) | (abs_vr >> 7 & 0x0f);
    tx[num  + 21] = 0x7f & power;
    tx[num  + 21] = use_dir? (tx[num  + 21] | 0x80) : tx[num  + 21];
}

namespace {
double Normalize(double angle)
{
    const double M_2PI = PARAM::Math::PI * 2;

    while( angle > PARAM::Math::PI ) {
        angle -= M_2PI;
    }

    while( angle <= -PARAM::Math::PI ) {
        angle += M_2PI;
    }
    return angle;
}

quint8 kickStandardization(int team, quint8 id, bool mode, quint16 power, double input_vel) {
    double new_power = 0;
    QString a, b, c;
    QString min_power, max_power;
    QString key = "";
    QSettings *read_ini = new QSettings("kickparam.ini", QSettings::IniFormat);
    key = QString("Robot%1/%2_a").arg(id).arg(mode ? "chip" : "flat");
    a = read_ini->value(key).toString();
    key = QString("Robot%1/%2_b").arg(id).arg(mode ? "chip" : "flat");
    b = read_ini->value(key).toString();
    key = QString("Robot%1/%2_c").arg(id).arg(mode ? "chip" : "flat");
    c = read_ini->value(key).toString();
    key = QString("Robot%1/%2_min").arg(id).arg(mode ? "chip" : "flat");
    min_power = read_ini->value(key).toString();
    key = QString("Robot%1/%2_max").arg(id).arg(mode ? "chip" : "flat");
    max_power = read_ini->value(key).toString();
    new_power = (a.toDouble() * power * power + b.toDouble() * power + c.toDouble());
    new_power = (quint8)(std::max(min_power.toDouble(), std::min(new_power, max_power.toDouble())));
    if (mode) { // 用于画图
        ZActionModule::instance()->kick_param[id].cb = b.toDouble();
        ZActionModule::instance()->kick_param[id].cc = c.toDouble();
    }
    else {
        ZActionModule::instance()->kick_param[id].fb = b.toDouble();
        ZActionModule::instance()->kick_param[id].fc = c.toDouble();
    }
//    qDebug() << "a:" << a.toDouble() << " b:" << b.toDouble() << " c:" << c.toDouble();
//    qDebug() << "id : " << id << " power : " << power << "new_power : " << new_power;
//    期望的绝对速度hope
    double vel_hope = input_vel;
//    实际的绝对速度real
    double vel_real = GlobalData::instance()->maintain[0].ball->velocity.mod();
    double kick_kp = power;
    double kick_hope = new_power;
//    记录数据flatshoot
    std::ofstream ratio_file("/home/zjunlict-vision-1/Desktop/dhz/Kun2/ZBin/data/feedbackData.txt", std::ios::app);
    if(ratio_file.is_open() && !mode) {
        ratio_file << " " << id << " " << mode << " " << vel_hope << " " << vel_real << " " << kick_kp << " " << kick_hope << std::endl;
        ratio_file.close();
        qDebug() << " " << id << " " << mode << " " << vel_hope << " " << vel_real << " " << kick_kp << " " << kick_hope;
    }
    return new_power;
}
} // namespace ZSS::anonymous
}
