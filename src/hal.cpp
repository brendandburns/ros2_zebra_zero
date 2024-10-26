#include "hal.h"

std::shared_ptr<zebra_zero::HardwareAbstractionLayer> zebra_zero::HardwareAbstractionLayer::instance_;

zebra_zero::HardwareAbstractionLayer::HardwareAbstractionLayer() {}
zebra_zero::HardwareAbstractionLayer::~HardwareAbstractionLayer() {}

std::shared_ptr<zebra_zero::HardwareAbstractionLayer> zebra_zero::HardwareAbstractionLayer::instance() {
    return instance_;
}