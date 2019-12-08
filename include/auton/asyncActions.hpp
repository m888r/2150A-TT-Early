#pragma once

#include <functional>
#define Async(code) RRLib::doAsync([]() code);

namespace RRLib {
void doAsync(std::function<void()>);
}
