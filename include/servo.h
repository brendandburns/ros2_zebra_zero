#ifndef __SERVO_H__
#define __SERVO_H__

#include <vector>

#include "module.h"

class Servo : public Module {
    public:
        Servo(int index);
        ~Servo();

        int read();
        void write(int pos, int velocity, int acceleration);
        bool zero();

        void initPath();
        bool setPath(const std::vector<long> &path);
};

#endif // __SERVO_H__