#include "servo.h"

#include "nmccom.h"
#include "picservo.h"

#include "rclcpp/rclcpp.hpp"

Servo::Servo(int ix) : Module(ix, ModuleType::SERVO)
{
    // Retrieve the position data from the local data structure
    NmcDefineStatus(this->_index, SEND_POS);

    ServoSetGain(this->_index, // axis = 1
                 100,          // Kp = 100
                 1000,         // Kd = 1000
                 0,            // Ki = 0
                 0,            // IL = 0
                 255,          // OL = 255
                 0,            // CL = 0
                 4000,         // EL = 4000
                 1,            // SR = 1
                 0             // DC = 0
    );

    ServoStopMotor(this->_index, AMP_ENABLE | MOTOR_OFF);   // enable amp
    ServoStopMotor(this->_index, AMP_ENABLE | STOP_ABRUPT); // stop at current pos.
    ServoResetPos(this->_index);
}

Servo::~Servo() {
}

int Servo::read()
{
    NmcNoOp(this->_index);
    return ServoGetPos(this->_index);
}

void Servo::write(int pos, int velocity, int acceleration)
{
    byte statbyte = 0;
    NmcNoOp(1);	//poll controller to get current status data
    statbyte = NmcGetStat(1);
    if (!(statbyte & MOVE_DONE)) {
        // If we're currently moving skip.
        // TODO: if the position has changed cancel the move and restart.
        return;
    }
    // RCLCPP_INFO(rclcpp::get_logger("ZebraZeroHardware"), "Setting position to %d for %d", pos, this->_index);
    ServoLoadTraj(this->_index, // addr = _index
                  LOAD_POS | LOAD_VEL | LOAD_ACC | ENABLE_SERVO | START_NOW,
                  pos,
                  velocity,
                  acceleration,
                  0
    );
}

bool Servo::zero()
{
    return ServoResetPos(this->_index);
}

void Servo::initPath()
{
    ServoInitPath(this->_index);
}

bool Servo::setPath(const std::vector<long> &path) {
    if (path.size() > 128) {
        return false;
    }
    if (path.size() == 0) {
        return true;
    }
    for (size_t i = 0; i < path.size(); i += 7) {
        bool ok = ServoAddPathpoints(this->_index, 7, (long *)&path[i], P_30HZ);
        if (!ok) {
            return false;
        }
    }
    if (path.size() % 7 != 0) {
        int ix = (path.size() / 7) * 7;
        if (! ServoAddPathpoints(this->_index, path.size() % 7, (long *)&path[ix], P_30HZ)) {
            return false;
        }
    }
    return true;
}