#ifndef __HAL_H__
#define __HAL_H__

#include <memory>
#include <vector>

namespace zebra_zero {
    class HardwareAbstractionLayer {
        private:
            static std::shared_ptr<HardwareAbstractionLayer> instance_;

        public:
            static std::shared_ptr<HardwareAbstractionLayer> instance();
            static void set_instance(HardwareAbstractionLayer* hal) { instance_.reset(hal); }

            HardwareAbstractionLayer();
            virtual ~HardwareAbstractionLayer() = 0;

            virtual void Shutdown() = 0;
            virtual uint8_t Init(char* path, int baudrate) = 0;
            virtual uint8_t GetModType(uint8_t addr) = 0;
            virtual bool DefineStatus(uint8_t addr, int flags) = 0;
            virtual void NoOp(uint8_t addr) = 0;

            virtual void InitPath(uint8_t addr) = 0;
            virtual bool StartPathMode(uint8_t group_addr, uint8_t leader_addr) = 0;
            virtual bool AddPathpoints(uint8_t addr, size_t num, long* pts) = 0;
            virtual bool ResetPos(uint8_t addr) = 0;
            virtual bool LoadTraj(uint8_t addr, uint8_t flags, int32_t position, uint32_t velocity, uint32_t acceleration, uint8_t pwm) = 0;
            virtual bool StopMotor(uint8_t addr, uint8_t flags) = 0;
            virtual long GetPos(uint8_t addr) = 0;
            virtual void GetPosAndVel(uint8_t addr, int* pos, int* vel) = 0;
            virtual bool SetGain(uint8_t addr, long Kp, long Kd, long Ki, long IL, long OL, long CL, long EL, long SR, long DC) = 0;

            virtual bool Moving(uint8_t addr) = 0;
    };
}

#endif // __HAL_H__
