#include "servo.h"

#include "nmccom.h"
#include "picservo.h"

Servo::Servo(int ix) : Module(ix, ModuleType::SERVO) {
    //Retrieve the position data from the local data structure
    NmcDefineStatus(this->_index, SEND_POS);
}

int Servo::read() {
    NmcNoOp(this->_index);
    return ServoGetPos(this->_index);
}

void Servo::write(int pos) {
    // TODO: something here...
    // printf("Setting position to %d\n", pos);
}
