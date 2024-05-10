#ifndef __STEPPER_H__
#define __STEPPER_H__

#include "module.h"

class Stepper : public Module {
    public:
        Stepper(int index) : Module(index, ModuleType::STEPPER) {}
        ~Stepper() {}

        void deactivate() override {}
        bool moving() override { return false; }
};

#endif // __STEPPER_H__