#include "real-hardware.h"
#include "constants.h"
#include <string>
#include <memory>

#include "nmc/driver.h"
#include "nmc/servo.h"
#include "nmc/logging.h"

using namespace nmc;
using zebra_zero::RealHardwareAbstractionLayer;

void RealHardwareAbstractionLayer::Shutdown()
{
    bus->hardReset();
    delete bus;
    bus = NULL;
}

uint8_t RealHardwareAbstractionLayer::Init(char *path, int) {
    bus = new nmc::Nmc(std::string(path));
    nmc::NmcLogging::setLevel(INFO_LEVEL);
    return bus->init();
}

uint8_t RealHardwareAbstractionLayer::GetModType(uint8_t addr) {
    return bus->getModule(addr)->getType();
}

bool RealHardwareAbstractionLayer::DefineStatus(uint8_t addr, int flags) {
    std::shared_ptr<Module> module = bus->getModule(addr);
    return static_cast<ServoModule*>(module.get())->defineStatus(flags);
}

void RealHardwareAbstractionLayer::NoOp(uint8_t addr) {
    std::shared_ptr<Module> module = bus->getModule(addr);
    static_cast<ServoModule*>(module.get())->noop();
}

void RealHardwareAbstractionLayer::InitPath(uint8_t addr) {
    std::shared_ptr<Module> module = bus->getModule(addr);
    static_cast<ServoModule*>(module.get())->initPath();
}

bool RealHardwareAbstractionLayer::StartPathMode(uint8_t group_addr, uint8_t leader_addr) {
    std::shared_ptr<Module> module = bus->getModule(leader_addr);
    return static_cast<ServoModule*>(module.get())->startPathMode(group_addr);
}

bool RealHardwareAbstractionLayer::AddPathpoints(uint8_t addr, size_t num, long *pts) {
    auto points = std::vector<int16_t>(num);
    for (size_t i = 0; i < num; i++) {
        points[i] = pts[i];
    }
    std::shared_ptr<Module> module = bus->getModule(addr);
    auto result = static_cast<ServoModule*>(module.get())->addPathPoints(points, 60 /* Hz */);
    return result;
}

bool RealHardwareAbstractionLayer::ResetPos(uint8_t addr) {
    std::shared_ptr<Module> module = bus->getModule(addr);
    return static_cast<ServoModule*>(module.get())->zero();
}

bool RealHardwareAbstractionLayer::LoadTraj(uint8_t addr, uint8_t flags, int32_t position, uint32_t velocity, uint32_t acceleration, uint8_t pwm) {
    std::shared_ptr<Module> module = bus->getModule(addr);
    return static_cast<ServoModule*>(module.get())->loadTrajectory(flags, position, velocity, acceleration, pwm);
}

bool RealHardwareAbstractionLayer::StopMotor(uint8_t addr, uint8_t flags) {
    std::shared_ptr<Module> module = bus->getModule(addr);
    return static_cast<ServoModule*>(module.get())->stop(flags);
}

long RealHardwareAbstractionLayer::GetPos(uint8_t addr) {
    std::shared_ptr<Module> module = bus->getModule(addr);
    return static_cast<ServoModule*>(module.get())->getPos();
}

void RealHardwareAbstractionLayer::GetPosAndVel(uint8_t addr, int* pos, int* vel) {
    std::shared_ptr<Module> module = bus->getModule(addr);
    *pos = static_cast<ServoModule*>(module.get())->getPos();
    // Velocity is in ticks / servo cycle, and the servo runs at 1953 Hz, translate to clicks/sec.
    *vel = 1953 * static_cast<ServoModule*>(module.get())->getVel();
}

bool RealHardwareAbstractionLayer::SetGain(uint8_t addr, long Kp, long Kd, long Ki, long IL, long OL, long CL, long EL, long SR, long DC) {
    std::shared_ptr<Module> module = bus->getModule(addr);
    return static_cast<ServoModule*>(module.get())->setGain(Kp, Kd, Ki, IL, OL, CL, EL, SR, DC);
}

bool RealHardwareAbstractionLayer::Moving(uint8_t addr) {
    uint8_t statbyte = 0;
    std::shared_ptr<Module> module = bus->getModule(addr);

    if (!static_cast<ServoModule*>(module.get())->noop()) {
        return false;
    }
    statbyte = static_cast<ServoModule*>(module.get())->getStat();
    return !(statbyte & MOVE_DONE);
}
