#include "mock-hardware.h"
#include <string>
#include <memory>

#include "mock.h"

using zebra_zero::HardwareAbstractionLayerMock;
using zebra_zero::MockHardwareAbstractionLayer;

std::shared_ptr<HardwareAbstractionLayerMock> HardwareAbstractionLayerMock::instance() {
    static std::shared_ptr<HardwareAbstractionLayerMock> i{ instantiate() };
    
    return i;
}

MockHardwareAbstractionLayer::MockHardwareAbstractionLayer() {}

void MockHardwareAbstractionLayer::Shutdown()
{
    HardwareAbstractionLayerMock::instance()->Shutdown();
}

uint8_t MockHardwareAbstractionLayer::Init(char *path, int baudrate) {
    return HardwareAbstractionLayerMock::instance()->Init(path, baudrate);
}

uint8_t MockHardwareAbstractionLayer::GetModType(uint8_t addr) {
    return HardwareAbstractionLayerMock::instance()->GetModType(addr);
}

bool MockHardwareAbstractionLayer::DefineStatus(uint8_t addr, int flags) {
    return HardwareAbstractionLayerMock::instance()->DefineStatus(addr, flags);
}

void MockHardwareAbstractionLayer::NoOp(uint8_t addr) {
    return HardwareAbstractionLayerMock::instance()->NoOp(addr);
}

void MockHardwareAbstractionLayer::InitPath(uint8_t  addr) {
    return HardwareAbstractionLayerMock::instance()->InitPath(addr);
}

bool MockHardwareAbstractionLayer::StartPathMode(uint8_t addr, uint8_t leader_addr) {
    return HardwareAbstractionLayerMock::instance()->StartPathMode(addr, leader_addr);
}

bool MockHardwareAbstractionLayer::AddPathpoints(uint8_t addr, size_t npoints, long* points) {
    return HardwareAbstractionLayerMock::instance()->AddPathpoints(addr, npoints, points);
}

bool MockHardwareAbstractionLayer::ResetPos(uint8_t addr) {
    return HardwareAbstractionLayerMock::instance()->ResetPos(addr);
}

bool MockHardwareAbstractionLayer::LoadTraj(uint8_t addr, uint8_t flags, int32_t pos, uint32_t vel, uint32_t acc, uint8_t pwm) {
    return HardwareAbstractionLayerMock::instance()->LoadTraj(addr, flags, pos, vel, acc, pwm);
}

bool MockHardwareAbstractionLayer::StopMotor(uint8_t addr, uint8_t flags) {
    return HardwareAbstractionLayerMock::instance()->StopMotor(addr, flags);
}

long MockHardwareAbstractionLayer::GetPos(uint8_t addr) {
    return HardwareAbstractionLayerMock::instance()->GetPos(addr);
}

void MockHardwareAbstractionLayer::GetPosAndVel(uint8_t addr, int* pos, int* vel) {
    HardwareAbstractionLayerMock::instance()->GetPosAndVel(addr, pos, vel);
}

bool MockHardwareAbstractionLayer::SetGain(uint8_t addr, long kp, long ki, long kd, long il, long ol, long cl, long el, long sr, long dc) {
    return HardwareAbstractionLayerMock::instance()->SetGain(addr, kp, ki, kd, il, ol, cl, el, sr, dc);
}

bool MockHardwareAbstractionLayer::Moving(uint8_t addr) {
    return HardwareAbstractionLayerMock::instance()->Moving(addr);
}
