#include "nmc.h"
#include "servo.h"

#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>

int main(int argc, char **argv)
{
    unsigned int baudrate = atoi(argv[2]);

    NmcBus bus(argv[1], baudrate);
    bus.init();
    for (int i = 0; i < 10; i++)
    {
        for (auto module : bus.modules())
        {
            switch (module->type())
            {
            case ModuleType::SERVO:
            {
                fprintf(stdout, "Servo found at %d\n", module->index());
                auto pos = ((Servo *)module)->read();
                fprintf(stdout, "Position is %d\n", pos);
            }
            break;
            case ModuleType::STEPPER:
                fprintf(stdout, "Stepper found at %d\n", module->index());
                break;
            }
        }
        sleep(1);
    }
}   
