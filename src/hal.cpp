#include "hal.h"
#include <string>
#include <memory>

using zebra_zero::HardwareAbstractionLayer;

std::shared_ptr<HardwareAbstractionLayer> HardwareAbstractionLayer::instance() {
    static std::shared_ptr<HardwareAbstractionLayer> i{ instantiate() };
    
    return i;
}


#ifndef MOCK_HARDWARE
#ifdef OLDNMC
void HardwareAbstractionLayer::Shutdown()
{
    NmcHardReset(0xFF);
    NmcShutdown();
}

uint8_t HardwareAbstractionLayer::Init(char *path, int baudrate) {
    return NmcInit(path, baudrate);
}

uint8_t HardwareAbstractionLayer::GetModType(uint8_t addr) {
    NmcReadStatus(addr, SEND_ID);
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
    // Velocity is in ticks / servo cycle, and the servo runs at 1953 Hz, translate to clicks/sec.
    *vel = 1953 * ServoGetVel(addr);
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
# else

#include "nmc/driver.h"
#include "nmc/servo.h"
#include "nmc/logging.h"

using namespace nmc;

void HardwareAbstractionLayer::Shutdown()
{
    bus->hardReset();
    delete bus;
    bus = NULL;
}

uint8_t HardwareAbstractionLayer::Init(char *path, int) {
    bus = new nmc::Nmc(std::string(path));
    nmc::NmcLogging::setLevel(INFO_LEVEL);
    return bus->init();
}

uint8_t HardwareAbstractionLayer::GetModType(uint8_t addr) {
    return bus->getModule(addr)->getType();
}

bool HardwareAbstractionLayer::DefineStatus(uint8_t addr, int flags) {
    return ((ServoModule*)bus->getModule(addr))->defineStatus(flags);
}

void HardwareAbstractionLayer::NoOp(uint8_t addr) {
    ((ServoModule*)bus->getModule(addr))->noop();
}

void HardwareAbstractionLayer::InitPath(uint8_t addr) {
    ((ServoModule*)bus->getModule(addr))->initPath();
}

bool HardwareAbstractionLayer::StartPathMode(uint8_t group_addr, uint8_t leader_addr) {
    return ((ServoModule*)bus->getModule(leader_addr))->startPathMode(group_addr);
}

bool HardwareAbstractionLayer::AddPathpoints(uint8_t addr, size_t num, long *pts) {
    auto points = new int16_t[num];
    for (size_t i = 0; i < num; i++) {
        points[i] = pts[i];
    }
    auto result = ((ServoModule*)bus->getModule(addr))->addPathPoints(num, points);
    delete [] points;
    return result;
}

bool HardwareAbstractionLayer::ResetPos(uint8_t addr) {
    return ((ServoModule*)bus->getModule(addr))->zero();
}

bool HardwareAbstractionLayer::LoadTraj(uint8_t addr, uint8_t flags, int32_t position, uint32_t velocity, uint32_t acceleration, uint8_t pwm) {
    return ((ServoModule*)bus->getModule(addr))->loadTrajectory(flags, position, velocity, acceleration, pwm);
}

bool HardwareAbstractionLayer::StopMotor(uint8_t addr, uint8_t flags) {
    return ((ServoModule*)bus->getModule(addr))->stop(flags);
}

long HardwareAbstractionLayer::GetPos(uint8_t addr) {
    return ((ServoModule*)bus->getModule(addr))->getPos();
}

void HardwareAbstractionLayer::GetPosAndVel(uint8_t addr, int* pos, int* vel) {
    *pos = ((ServoModule*)bus->getModule(addr))->getPos();
    // Velocity is in ticks / servo cycle, and the servo runs at 1953 Hz, translate to clicks/sec.
    *vel = 1953 * ((ServoModule*)bus->getModule(addr))->getVel();
}

bool HardwareAbstractionLayer::SetGain(uint8_t addr, long Kp, long Kd, long Ki, long IL, long OL, long CL, long EL, long SR, long DC) {
    return ((ServoModule*)bus->getModule(addr))->setGain(Kp, Kd, Ki, IL, OL, CL, EL, SR, DC);
}

bool HardwareAbstractionLayer::Moving(uint8_t addr) {
    uint8_t statbyte = 0;
    if (!((ServoModule*)bus->getModule(addr))->noop()) {
        return false;
    }
    statbyte = ((ServoModule*)bus->getModule(addr))->getStat();
    return !(statbyte & MOVE_DONE);
}
#endif // OLDNMC
#else

#include "mock.h"

using zebra_zero::HardwareAbstractionLayerMock;

std::shared_ptr<HardwareAbstractionLayerMock> HardwareAbstractionLayerMock::instance() {
    static std::shared_ptr<HardwareAbstractionLayerMock> i{ instantiate() };
    
    return i;
}


void HardwareAbstractionLayer::Shutdown()
{
    HardwareAbstractionLayerMock::instance()->Shutdown();
}

uint8_t HardwareAbstractionLayer::Init(char *path, int baudrate) {
    return HardwareAbstractionLayerMock::instance()->Init(path, baudrate);
}

uint8_t HardwareAbstractionLayer::GetModType(uint8_t addr) {
    return HardwareAbstractionLayerMock::instance()->GetModType(addr);
}

bool HardwareAbstractionLayer::DefineStatus(uint8_t addr, int flags) {
    return HardwareAbstractionLayerMock::instance()->DefineStatus(addr, flags);
}

void HardwareAbstractionLayer::NoOp(uint8_t addr) {
    return HardwareAbstractionLayerMock::instance()->NoOp(addr);
}

void HardwareAbstractionLayer::InitPath(uint8_t  addr) {
    return HardwareAbstractionLayerMock::instance()->InitPath(addr);
}

bool HardwareAbstractionLayer::StartPathMode(uint8_t addr, uint8_t leader_addr) {
    return HardwareAbstractionLayerMock::instance()->StartPathMode(addr, leader_addr);
}

bool HardwareAbstractionLayer::AddPathpoints(uint8_t addr, size_t npoints, long* points) {
    return HardwareAbstractionLayerMock::instance()->AddPathpoints(addr, npoints, points);
}

bool HardwareAbstractionLayer::ResetPos(uint8_t addr) {
    return HardwareAbstractionLayerMock::instance()->ResetPos(addr);
}

bool HardwareAbstractionLayer::LoadTraj(uint8_t addr, int flags, long pos, long vel, long acc, long pwm) {
    return HardwareAbstractionLayerMock::instance()->LoadTraj(addr, flags, pos, vel, acc, pwm);
}

bool HardwareAbstractionLayer::StopMotor(uint8_t addr, int flags) {
    return HardwareAbstractionLayerMock::instance()->StopMotor(addr, flags);
}

long HardwareAbstractionLayer::GetPos(uint8_t addr) {
    return HardwareAbstractionLayerMock::instance()->GetPos(addr);
}

void HardwareAbstractionLayer::GetPosAndVel(uint8_t addr, int* pos, int* vel) {
    HardwareAbstractionLayerMock::instance()->GetPosAndVel(addr, pos, vel);
}

bool HardwareAbstractionLayer::SetGain(uint8_t addr, long kp, long ki, long kd, long il, long ol, long cl, long el, long sr, long dc) {
    return HardwareAbstractionLayerMock::instance()->SetGain(addr, kp, ki, kd, il, ol, cl, el, sr, dc);
}

bool HardwareAbstractionLayer::Moving(uint8_t addr) {
    return HardwareAbstractionLayerMock::instance()->Moving(addr);
}

#endif // LIBNMC
