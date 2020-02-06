#pragma once
#include "config.hpp"
#include "okapi/api.hpp"
//#include "tray.hpp"

namespace subsystem {
namespace intake {

void init();

void run(void* p);

enum class state { manual, free, in, out, outPosition, placing, holding, disabled };

void in();

void out();

void free();

void manual();

void changeState(state state);

void outAtSpeed(double speed);
}  // namespace intake
}  // namespace subsystem