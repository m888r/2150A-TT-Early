#pragma once

namespace subsystem {
namespace rd4b {
enum class Mode { motionprofile, pid };

void run(void* p);
void moveTargetMotionProfile(double target);
void moveTargetPID(double target);

}  // namespace rd4b
}  // namespace subsystem