#include "hal.h"

using zebra_zero::HardwareAbstractionLayer;

std::shared_ptr<HardwareAbstractionLayer> HardwareAbstractionLayer::instance() {
    static std::shared_ptr<HardwareAbstractionLayer> i{ instantiate() };
    
    return i;
}


#ifndef MOCK_HARDWARE

void HardwareAbstractionLayer::Shutdown()
{
    NmcShutdown();
}

uint8_t HardwareAbstractionLayer::Init(char *path, int baudrate) {
    return NmcInit(path, baudrate);
}

uint8_t HardwareAbstractionLayer::GetModType(uint8_t addr) {
    return NmcGetModType(addr);
}

bool HardwareAbstractionLayer::DefineStatus(uint8_t addr, int flags) {
    return NmcDefineStatus(addr, flags);
}

void HardwareAbstractionLayer::NoOp(uint8_t addr) {
    NmcNoOp(addr);
}

void HardwareAbstractionLayer::InitPath(uint8_t addr) {
    ServoInitPath(addr);
}

bool HardwareAbstractionLayer::StartPathMode(uint8_t group_addr, uint8_t leader_addr) {
    return ServoStartPathMode(group_addr, leader_addr);
}

bool HardwareAbstractionLayer::AddPathpoints(uint8_t addr, size_t num, long *pts) {
    return ServoAddPathpoints(addr, num, pts, P_30HZ);
}

bool HardwareAbstractionLayer::ResetPos(uint8_t addr) {
    return ServoResetPos(addr);
}

bool HardwareAbstractionLayer::LoadTraj(uint8_t addr, int flags, long position, long velocity, long acceleration, long pwm) {
    return ServoLoadTraj(addr, flags, position, velocity, acceleration, pwm);
}

bool HardwareAbstractionLayer::StopMotor(uint8_t addr, int flags) {
    return ServoStopMotor(addr, flags);
}

long HardwareAbstractionLayer::GetPos(uint8_t addr) {
    return ServoGetPos(addr);
}

void HardwareAbstractionLayer::GetPosAndVel(uint8_t addr, int* pos, int* vel) {
    *pos = ServoGetPos(addr);
    *vel = ServoGetVel(addr);
}

bool HardwareAbstractionLayer::SetGain(uint8_t addr, long Kp, long Kd, long Ki, long IL, long OL, long CL, long EL, long SR, long DC) {
    return ServoSetGain(addr, Kp, Kd, Ki, IL, OL, CL, EL, SR, DC);
}

bool HardwareAbstractionLayer::Moving(uint8_t addr) {
    byte statbyte = 0;
    NmcNoOp(1);
    statbyte = NmcGetStat(addr);
    return !(statbyte & MOVE_DONE);
}

#else

#include <cstdio>

void HardwareAbstractionLayer::Shutdown()
{
    printf("Shutdown!\n");
}

uint8_t HardwareAbstractionLayer::Init(char *path, int baudrate) {
    printf("init called for %s @ %d\n", path, baudrate);
    return 0;
}

uint8_t HardwareAbstractionLayer::GetModType(uint8_t) {
    return 0;
}

bool HardwareAbstractionLayer::DefineStatus(uint8_t, int) {
    return false;
}

void HardwareAbstractionLayer::NoOp(uint8_t) {
    // pass
}

void HardwareAbstractionLayer::InitPath(uint8_t) {
    // pass
}

bool HardwareAbstractionLayer::StartPathMode(uint8_t, uint8_t) {
    return false;
}

bool HardwareAbstractionLayer::AddPathpoints(uint8_t, size_t, long*) {
    return false;
}

bool HardwareAbstractionLayer::ResetPos(uint8_t) {
    return false;
}

bool HardwareAbstractionLayer::LoadTraj(uint8_t, int, long, long, long, long) {
    return false;
}

bool HardwareAbstractionLayer::StopMotor(uint8_t, int) {
    return false;
}

long HardwareAbstractionLayer::GetPos(uint8_t) {
    return 0;
}

void HardwareAbstractionLayer::GetPosAndVel(uint8_t, int*, int*) {
}

bool HardwareAbstractionLayer::SetGain(uint8_t, long, long, long, long, long, long, long, long, long) {
    return false;
}

bool HardwareAbstractionLayer::Moving(uint8_t) {
    return false;
}

#endif // LIBNMC
