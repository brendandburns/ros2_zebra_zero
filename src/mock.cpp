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

uint8_t HardwareAbstractionLayerMock::Init(char *, int) {
    return 6;
}

uint8_t HardwareAbstractionLayerMock::GetModType(uint8_t) {
    return SERVOMODTYPE;
}

bool HardwareAbstractionLayerMock::DefineStatus(uint8_t, int) {
    return true;
}

void HardwareAbstractionLayerMock::NoOp(uint8_t) {
}

void HardwareAbstractionLayerMock::InitPath(uint8_t) {
}

bool HardwareAbstractionLayerMock::StartPathMode(uint8_t, uint8_t) {
    return true;
}

bool HardwareAbstractionLayerMock::AddPathpoints(uint8_t, size_t, long*) {
    return true;
}

bool HardwareAbstractionLayerMock::ResetPos(uint8_t addr) {
    this->pos[addr] = 0;
    return true;
}

bool HardwareAbstractionLayerMock::LoadTraj(uint8_t addr, int flags, long pos, long, long, long) {
    if (flags & LOAD_POS) {
        this->pos[addr] = pos;
    }
    return true;
}

bool HardwareAbstractionLayerMock::StopMotor(uint8_t, int) {
    return true;
}

long HardwareAbstractionLayerMock::GetPos(uint8_t addr) {
    return pos[addr];
}

void HardwareAbstractionLayerMock::GetPosAndVel(uint8_t addr, int* pos, int* vel) {
    *pos = this->pos[addr];
    *vel = 0;
}

bool HardwareAbstractionLayerMock::SetGain(uint8_t, long, long, long, long, long, long, long, long, long) {
    return true;
}

bool HardwareAbstractionLayerMock::Moving(uint8_t) {
    return false;
}