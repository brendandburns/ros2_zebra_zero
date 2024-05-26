#ifndef __SERVO_H__
#define __SERVO_H__

#include <vector>

#include "module.h"

class Servo : public Module {
    public:
        Servo(int index);
        ~Servo();

        void read(int *pos, int* velocity);
        void write(int pos, int velocity, int acceleration);
        void velocity(int velocity, int acceleration);
        void pwm(int pwm);
        bool stop(bool smooth);

        bool zero();

        void initPath();
        bool setPath(const std::vector<long> &path);

        void deactivate() override;
        bool moving() override;
    
    private:
        bool _active;
};

#endif // __SERVO_H__