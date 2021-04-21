#ifndef ZACTIONMODULE_H
#define ZACTIONMODULE_H
#include <QObject>
#include <QMutex>
#include <QUdpSocket>
#include "singleton.hpp"
#include "zss_cmd.pb.h"
#include "staticparams.h"
namespace ZSS {
class ActionModule : public QObject {
    Q_OBJECT
  public:
    ActionModule(QObject *parent = 0);
    ~ActionModule();
    void sendLegacy(int t, const ZSS::Protocol::Robots_Command&);
    bool connectRadio(int, int);
    bool disconnectRadio(int);
    void setSimulation(bool);
    int team[PARAM::TEAMS];
    void changeAddress(int team,int index);
    QStringList getAllAddress();
    QString getRealAddress(int team);
    int getAddressNum();
    void encodeLegacy(const ZSS::Protocol::Robot_Command& command, QByteArray& tx, int num, int team);
    void saveData(int team, int id, qint64 time, short*);
    void saveCommand(int id, qint64 time, double vx, double vy, double vr);
    int reportVec[PARAM::TEAMS];
    qint64 last_receive_time[PARAM::TEAMS][PARAM::ROBOTNUM];
    bool no_response[PARAM::TEAMS][PARAM::ROBOTNUM];
  private slots:
    void readData();
  private:
    void sendStartPacket(int, int);
  private:
    QByteArray tx;
    QByteArray rx;
    QUdpSocket sendSocket;
    QUdpSocket receiveSocket;
    QMutex sendMutex;
  signals:
    void receiveRobotInfo(int, int);
};
typedef Singleton<ActionModule> ZActionModule;
}
#endif // ZACTIONMODULE_H
