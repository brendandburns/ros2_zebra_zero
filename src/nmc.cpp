#include "nmc.h"
#include "servo.h"
#include "stepper.h"
#include "nmccom.h"

#include <string.h>
#include <stdlib.h>

NmcBus::NmcBus(const char* port, unsigned int baudrate) : _port(strdup(port)), _baudrate(baudrate) {}

NmcBus::~NmcBus() {
    free(this->_port);
    NmcShutdown();
}

unsigned int NmcBus::init() {
    auto count = NmcInit(this->_port, this->_baudrate);
    for (int i = 0; i < count; i++) {
        auto mod = NmcGetModType((byte) i + 1);
        switch(mod) {
            case SERVOMODTYPE:
                this->_modules.push_back(new Servo(i+1));
                break;
            case STEPMODTYPE:
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
