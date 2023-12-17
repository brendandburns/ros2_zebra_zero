#ifndef __MODULE_H__
#define __MODULE_H__

enum ModuleType {
    SERVO,
    STEPPER
};

class Module {
    protected:
        int _index;
        ModuleType _type;

    public:
        Module(int index, ModuleType type) : _index(index), _type(type) {}
        virtual ~Module() {}

        int index() { return this->_index; }
        ModuleType type() { return this->_type; }
};

#endif