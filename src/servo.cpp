#include "servo.h"

#include "hal.h"
#include "rclcpp/rclcpp.hpp"

using zebra_zero::HardwareAbstractionLayer;

Servo::Servo(int ix) : Module(ix, ModuleType::SERVO), _active(false)
{
    // Retrieve the position data from the local data structure
    HardwareAbstractionLayer::instance()->DefineStatus(this->_index, SEND_POS | SEND_VEL); // | SEND_NPOINTS | SEND_PERROR | SEND_AUX);


    HardwareAbstractionLayer::instance()->SetGain(this->_index, // axis = 1
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

    HardwareAbstractionLayer::instance()->StopMotor(this->_index, AMP_ENABLE | MOTOR_OFF);   // enable amp
    HardwareAbstractionLayer::instance()->StopMotor(this->_index, AMP_ENABLE | STOP_ABRUPT | ADV_FEATURE); // stop at current pos.
    HardwareAbstractionLayer::instance()->ResetPos(this->_index);
    _active = true;
}

Servo::~Servo() {
    if (_active) {
        deactivate();
    }
}

void Servo::deactivate() {
    this->stop(false);
    HardwareAbstractionLayer::instance()->StopMotor(this->_index, MOTOR_OFF);
    _active = false;
}

void Servo::read(int* pos, int* vel)
{
    if (!_active) {
        return;
    }
    HardwareAbstractionLayer::instance()->NoOp(this->_index);
    HardwareAbstractionLayer::instance()->GetPosAndVel(this->_index, pos, vel);
}

void Servo::write(int pos, int velocity, int acceleration)
{
    if (!_active) {
        return;
    }
    if (this->moving()) {
        // If we're currently moving skip.
        // TODO: if the position has changed cancel the move and restart.
        return;
    }
    // RCLCPP_INFO(rclcpp::get_logger("ZebraZeroHardware"), "Setting position to %d for %d", pos, this->_index);
    
    // See section 4.4.7 of the PIC-SERVO manual. The lower 16 bits are treated as fractional components
    // we'll ignore them for now because we definitely don't need to move that slow.
    // The formula is 2^16/2000 because the pic servo goes at 2khz, 2^16/2000 ~=32 (close enough...)
    // velocity = velocity << 5;
    // acceleration = acceleration << 5;

    HardwareAbstractionLayer::instance()->LoadTraj(this->_index, // addr = _index
                  LOAD_POS | LOAD_VEL | LOAD_ACC | ENABLE_SERVO | START_NOW,
                  pos,
                  velocity,
                  acceleration,
                  0
    );
}

bool Servo::stop(bool smooth)
{
    int mode = AMP_ENABLE | ADV_FEATURE | (smooth ? STOP_SMOOTH : STOP_ABRUPT);
    return HardwareAbstractionLayer::instance()->StopMotor(this->_index, mode); // stop at current pos.
}

void Servo::velocity(int velocity, int acceleration)
{
    if (!_active) {
        return;
    }
    if (this->moving()) {
        if (!this->stop(false)) {
            RCLCPP_ERROR(rclcpp::get_logger("ZebraZeroHardware"), "Failed to stop motor (%d)", this->_index);
            return;
        }
    }
    int mode = LOAD_VEL | LOAD_ACC | ENABLE_SERVO | VEL_MODE | START_NOW;
    if (velocity < 0) {
        velocity = -velocity;
        mode |= REVERSE;
    }
    // See section 4.4.7 of the PIC-SERVO manual. The lower 16 bits are treated as fractional components
    // we'll ignore them for now because we definitely don't need to move that slow.
    // The formula is 2^16/2000 because the pic servo goes at 2khz, 2^16/2000 ~=32 (close enough...)
    velocity = velocity << 5;
    acceleration = acceleration << 5;
    if (!HardwareAbstractionLayer::instance()->LoadTraj(this->_index, mode, 0, velocity, acceleration, 0))
    {
        RCLCPP_ERROR(rclcpp::get_logger("ZebraZeroHardware"), "Failed to set velocity (%d)", this->_index);
    }
}        

bool Servo::zero()
{
    if (!_active) {
        return false;
    }
    return HardwareAbstractionLayer::instance()->ResetPos(this->_index);
}

void Servo::initPath()
{
    if (!_active) {
        return;
    }
    HardwareAbstractionLayer::instance()->InitPath(this->_index);
}

bool Servo::setPath(const std::vector<long> &path) {
    if (!_active) {
        return false;
    }
    if (path.size() > 128) {
        return false;
    }
    if (path.size() == 0) {
        return true;
    }
    for (size_t i = 0; i < path.size(); i += 7) {
        bool ok = HardwareAbstractionLayer::instance()->AddPathpoints(this->_index, 7, (long *)&path[i]);
        if (!ok) {
            return false;
        }
    }
    if (path.size() % 7 != 0) {
        int ix = (path.size() / 7) * 7;
        if (! HardwareAbstractionLayer::instance()->AddPathpoints(this->_index, path.size() % 7, (long *)&path[ix])) {
            return false;
        }
    }
    return true;
}

bool Servo::moving() {
    return HardwareAbstractionLayer::instance()->Moving(this->_index);
    
    /*
    HardwareAbstractionLayer::instance()->NoOp(this->_index);
    byte statbyte = HardwareAbstractionLayer::instance()->GetStat(this->_index);
    return !(statbyte & MOVE_DONE);
    */
}