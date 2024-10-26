#ifndef __REAL_HAL_H__
#define __REAL_HAL_H__

#include "hal.h"
#include "nmc/driver.h"

namespace zebra_zero {
    class RealHardwareAbstractionLayer : public HardwareAbstractionLayer {
        public:
            nmc::Nmc* bus;
        
        public:
            RealHardwareAbstractionLayer() : bus(nullptr) {}
            ~RealHardwareAbstractionLayer() { Shutdown(); }

            void Shutdown() override;
            uint8_t Init(char* path, int baudrate) override;
            uint8_t GetModType(uint8_t addr) override;
            bool DefineStatus(uint8_t addr, int flags) override;
            void NoOp(uint8_t addr) override;

            void InitPath(uint8_t addr) override;
            bool StartPathMode(uint8_t group_addr, uint8_t leader_addr) override;
            bool AddPathpoints(uint8_t addr, size_t num, long* pts) override;
            bool ResetPos(uint8_t addr) override;
            bool LoadTraj(uint8_t addr, uint8_t flags, int32_t position, uint32_t velocity, uint32_t acceleration, uint8_t pwm) override;
            bool StopMotor(uint8_t addr, uint8_t flags) override;
            long GetPos(uint8_t addr) override;
            void GetPosAndVel(uint8_t addr, int* pos, int* vel) override;
            bool SetGain(uint8_t addr, long Kp, long Kd, long Ki, long IL, long OL, long CL, long EL, long SR, long DC) override;

            bool Moving(uint8_t addr) override;
    };
}

#endif // __HAL_H__
