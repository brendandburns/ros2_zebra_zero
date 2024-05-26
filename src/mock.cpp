#include "mock.h"
#include "rclcpp/rclcpp.hpp"

using zebra_zero::HardwareAbstractionLayerMock;

HardwareAbstractionLayerMock::HardwareAbstractionLayerMock()
{
    // NMC hardware indexes from 1, so allocate 7 slots and ignore slot 0
    this->pos.assign(7, 0);
}

void HardwareAbstractionLayerMock::Shutdown()
{
}

uint8_t HardwareAbstractionLayerMock::Init(char *path, int baudrate) {
    return 6;
}

uint8_t HardwareAbstractionLayerMock::GetModType(uint8_t addr) {
    return SERVOMODTYPE;
}

bool HardwareAbstractionLayerMock::DefineStatus(uint8_t addr, int flags) {
    return true;
}

void HardwareAbstractionLayerMock::NoOp(uint8_t addr) {
}

void HardwareAbstractionLayerMock::InitPath(uint8_t  addr) {
}

bool HardwareAbstractionLayerMock::StartPathMode(uint8_t addr, uint8_t leader_addr) {
    return true;
}

bool HardwareAbstractionLayerMock::AddPathpoints(uint8_t addr, size_t npoints, long* points) {
    return true;
}

bool HardwareAbstractionLayerMock::ResetPos(uint8_t addr) {
    this->pos[addr] = 0;
    return true;
}

bool HardwareAbstractionLayerMock::LoadTraj(uint8_t addr, int flags, long pos, long vel, long acc, long pwm) {
    if (flags & LOAD_POS) {
        this->pos[addr] = pos;
    }
    return true;
}

bool HardwareAbstractionLayerMock::StopMotor(uint8_t addr, int flags) {
    return true;
}

long HardwareAbstractionLayerMock::GetPos(uint8_t addr) {
    return pos[addr];
}

void HardwareAbstractionLayerMock::GetPosAndVel(uint8_t addr, int* pos, int* vel) {
    *pos = this->pos[addr];
    *vel = 0;
}

bool HardwareAbstractionLayerMock::SetGain(uint8_t addr, long kp, long ki, long kd, long il, long ol, long cl, long el, long sr, long dc) {
    return true;
}

bool HardwareAbstractionLayerMock::Moving(uint8_t addr) {
    return false;
}