#include "nmc.h"
#include "servo.h"
#include "stepper.h"
#include "hal.h"

#include <cstring>
#include <cstdlib>
#include <cstdio>

using zebra_zero::HardwareAbstractionLayer;

NmcBus::NmcBus(const char* port, unsigned int baudrate) : _port(strdup(port)), _baudrate(baudrate) {}

NmcBus::~NmcBus() {
    free(this->_port);
    for (Module *m: this->_modules) {
        m->deactivate();
        delete(m);
    }
    HardwareAbstractionLayer::instance()->Shutdown();
}

unsigned int NmcBus::init() {
    auto count = HardwareAbstractionLayer::instance()->Init(this->_port, this->_baudrate);
    for (auto i = 0; i < count; i++) {
        auto mod = HardwareAbstractionLayer::instance()->GetModType((uint8_t) i + 1);
        switch(mod) {
            case SERVOMODTYPE:
                this->_modules.push_back(new Servo(i+1));
                break;
            case STEPPERMODTYPE:
                this->_modules.push_back(new Stepper(i+1));
                break;
            default:
                fprintf(stderr, "Unknown module type: %d\n", mod);
        }
    }
    return count;
}

const std::vector<Module*>& NmcBus::modules() { return this->_modules; }
const Module* NmcBus::module(int ix) { return this->_modules[ix]; }

void NmcBus::initPath()
{
    for (size_t i = 0; i < this->_modules.size(); i++) {
        if (this->_modules[i]->type() == ModuleType::SERVO)
        {
            ((Servo*)this->_modules[i])->initPath();
        }
    }
}

bool NmcBus::startPath()
{
    return HardwareAbstractionLayer::instance()->StartPathMode(0xFF, 1);
}

bool NmcBus::moving() {
    for (Module *m : this->_modules) {
        if (m->moving()) {
            return true;
        }
    }
    return false;
}