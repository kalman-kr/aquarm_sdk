#include <kcan_sdk_user.h>

#include <chrono>
#include <csignal>
#include <cstdio>
#include <thread>
#include <vector>

constexpr float kTargetPosition = 0.0f;  // rad
constexpr int kNodeId = 6;
constexpr int kJointCount = 6;

static volatile std::sig_atomic_t running = 1;

int main() {
  std::signal(SIGINT, [](int) { running = 0; });

  kcan_user::KcanUserSDK sdk(kJointCount);
  if (!sdk.connect("192.168.1.9", 20000, 20001)) return 1;

  //User API sets position mode and starts sync
  if (!sdk.send_operational(kNodeId, 0.005f)) {
    sdk.send_stop();
    sdk.disconnect();
    return 1;
  }

  //Hold the other joints at their feedback positions
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  const auto states = sdk.get_joint_states();
  std::vector<float> command(kJointCount);
  for (int i = 0; i < kJointCount; ++i) command[i] = states[i].position;
  command[kNodeId - 1] = kTargetPosition;

  while (running) {
    sdk.send_joint_command(command);
    const auto s = sdk.get_joint_states()[kNodeId - 1];
    std::printf("\rpos %+.3f  vel %+.3f  torque %+.3f", s.position, s.velocity, s.torque);
    std::fflush(stdout);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  sdk.send_stop();
  sdk.disconnect();
  return 0;
}
