#pragma once
#ifndef KCAN_SDK_USER_H_
#define KCAN_SDK_USER_H_

#include <cstdint>
#include <string>
#include <vector>
#include <memory>

#if defined(_WIN32)
#if defined(KCAN_SDK_USER_BUILD)
#define KCAN_SDK_USER_API __declspec(dllexport)
#else
#define KCAN_SDK_USER_API __declspec(dllimport)
#endif
#elif defined(__GNUC__)
#define KCAN_SDK_USER_API __attribute__((visibility("default")))
#else
#define KCAN_SDK_USER_API
#endif

namespace kcan_user {

struct JointState {
  std::string name;
  float position{0.0f};
  float velocity{0.0f};
  float torque{0.0f};
};

class KCAN_SDK_USER_API KcanUserSDK {
public:
  explicit KcanUserSDK(int joint_count = 7);
  ~KcanUserSDK();
  bool connect(const std::string& robot_ip, int remote_port, int local_port);
  void disconnect();
  bool send_preoperational(uint8_t node_id = 0x00);
  bool send_operational(uint8_t node_id = 0x00,
                        float sync_period_sec = 0.002f);
  bool send_stop(uint8_t node_id = 0x00);
  bool send_joint_command(const std::vector<float>& commands);
  std::vector<JointState> get_joint_states() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace kcan_user
#endif
