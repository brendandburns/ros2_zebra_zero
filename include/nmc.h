#ifndef __NMC_H__
#define __NMC_H__

#include "module.h"
#include <vector>

class NmcBus {
    private:
        char *_port;
        unsigned int _baudrate;

        unsigned int  _num_modules;
        std::vector<Module*> _modules;

    public:
        NmcBus(const char* port, unsigned int baudrate);
        ~NmcBus();

        unsigned int init();
        const std::vector<Module *>& modules();
        const Module* module(int ix);
};

#endif // __NMC_H__