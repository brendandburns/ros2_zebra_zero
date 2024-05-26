#ifndef __MOCK_H__
#define __MOCK_H__

#include "constants.h"

#include <memory>
#include <vector>

namespace zebra_zero {
    class CommandRecord {
        private:
            int type;
            uint8_t addr;
        
        public:
            CommandRecord(int type, uint8_t addr) : type(type), addr(addr) {}
            int GetType() { return type; }
            uint8_t GetAddr() { return addr; }
    };

        class HardwareAbstractionLayerMock {
        public:
            static std::shared_ptr<HardwareAbstractionLayerMock> instance();

        private:
            static HardwareAbstractionLayerMock* instantiate() {
                return new HardwareAbstractionLayerMock();
            }

            HardwareAbstractionLayerMock();
        
            char *path;
            int baudrate;
            std::vector<CommandRecord> commands;
            std::vector<long> pos;
        
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
            bool LoadTraj(uint8_t addr, int flags, long position, long velocity, long acceleration, long pwm);
            bool StopMotor(uint8_t addr, int flags);
            long GetPos(uint8_t addr);
            void GetPosAndVel(uint8_t addr, int* pos, int* vel);
            bool SetGain(uint8_t addr, long Kp, long Kd, long Ki, long IL, long OL, long CL, long EL, long SR, long DC);

            bool Moving(uint8_t addr);
    };
}

#endif  // __MOCK_H__