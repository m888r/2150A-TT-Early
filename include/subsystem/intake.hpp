#pragma once
#include "config.hpp"
#include "okapi/api.hpp"
//#include "tray.hpp"

namespace subsystem {
namespace intake {

void init();

void run(void* p);

enum class state { manual, free, in, out, placing };

void in();

void out();

void free();

void manual();

void changeState(state state);
}  // namespace intake
}  // namespace subsystem