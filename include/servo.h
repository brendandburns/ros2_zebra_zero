#ifndef __SERVO_H__
#define __SERVO_H__

#include "module.h"

class Servo : public Module {
    public:
        Servo(int index);
        ~Servo() {}

        int read();
        void write(int pos);
};

#endif // __SERVO_H__