#pragma once
#include "config.hpp"
#include "okapi/api.hpp"

namespace subsystem {
namespace intake {

void init();

void run(void* p);

enum class state { manual, free, in, out };

void in();

void out();

void free();

void manual();
}  // namespace intake
}  // namespace subsystem