#ifndef __HAL_H__
#define __HAL_H__

#include "constants.h"
#include <memory>
#include <vector>

#ifdef MOCK_HARDWARE
namespace nmc
{
    class Nmc
    {
    };
}
#else
#include "nmc/driver.h"
#endif

namespace zebra_zero {
    class HardwareAbstractionLayer {
        public:
            static std::shared_ptr<HardwareAbstractionLayer> instance();
            nmc::Nmc* bus;

        private:
            static HardwareAbstractionLayer* instantiate() {
                return new HardwareAbstractionLayer();
            }

            HardwareAbstractionLayer() : bus(NULL) {}
        
        public:
            void Shutdown();
            uint8_t Init(char* path, int baudrate);
            uint8_t GetModType(uint8_t addr);
            bool DefineStatus(uint8_t addr, int flags);
            void NoOp(uint8_t addr);

            void InitPath(uint8_t addr);
            bool StartPathMode(uint8_t group_addr, uint8_t leader_addr);
            bool AddPathpoints(uint8_t addr, size_t num, long* pts);
            bool ResetPos(uint8_t addr);
            bool LoadTraj(uint8_t addr, uint8_t flags, int32_t position, uint32_t velocity, uint32_t acceleration, uint8_t pwm);
            bool StopMotor(uint8_t addr, uint8_t flags);
            long GetPos(uint8_t addr);
            void GetPosAndVel(uint8_t addr, int* pos, int* vel);
            bool SetGain(uint8_t addr, long Kp, long Kd, long Ki, long IL, long OL, long CL, long EL, long SR, long DC);

            bool Moving(uint8_t addr);
    };
}

#endif // __HAL_H__
