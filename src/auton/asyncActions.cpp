#include "main.h"

#include <list>

namespace RRLib {
  using namespace pros;
static std::list<std::function<void()>>& getAsyncMoveQueue() {
    static Mutex asyncQueueLock;
    static std::list<std::function<void()>> asyncMoveQueue;

    asyncQueueLock.take(TIMEOUT_MAX);
    auto& ret = asyncMoveQueue;
    asyncQueueLock.give();
    return ret;
}

static Task asyncMoveTask([](void*) {
    while (1) {
        getAsyncMoveQueue().clear();

        while (!competition::is_disabled() && asyncMoveTask.notify_take(0, TIMEOUT_MAX)) {
            auto func = getAsyncMoveQueue().front();
            getAsyncMoveQueue().pop_front();

            func();
            delay(1);
        }

        delay(1);
    }
}, nullptr, "ASYNC REEEEEEE");

void doAsync(std::function<void()> func) {
    getAsyncMoveQueue().push_back(func);
    asyncMoveTask.notify();
}
}
