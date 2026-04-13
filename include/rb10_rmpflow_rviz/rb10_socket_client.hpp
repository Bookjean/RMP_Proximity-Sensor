#pragma once

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

namespace rb10_rmpflow_rviz
{

struct Rb10SystemState
{
  float controller_time_sec{0.0F};
  std::array<float, 6> joint_ref_deg{};
  std::array<float, 6> joint_ang_deg{};
  std::int32_t robot_state{0};
  std::int32_t power_state{0};
  std::int32_t program_mode{0};
  std::int32_t collision_detect_onoff{0};
  std::int32_t init_state_info{0};
  std::int32_t init_error{0};
  std::int32_t op_stat_collision_occur{0};
  std::int32_t op_stat_sos_flag{0};
  std::int32_t op_stat_self_collision{0};
  std::int32_t op_stat_soft_estop_occur{0};
  std::int32_t op_stat_ems_flag{0};
};

class Rb10SocketClient
{
public:
  using StateCallback = std::function<void(const Rb10SystemState &)>;
  using LogCallback = std::function<void(const std::string &)>;

  Rb10SocketClient();
  ~Rb10SocketClient();

  bool connect(
    const std::string & robot_ip,
    double data_request_rate_hz,
    StateCallback state_callback,
    LogCallback log_callback,
    bool enable_realtime_threads = false,
    int realtime_priority = 0);
  void disconnect();

  bool initialize_robot(bool simulation_mode);
  bool send_movej_degrees(
    const std::array<double, 6> & joint_deg,
    double speed,
    double acceleration);
  bool send_servoj_degrees(
    const std::array<double, 6> & joint_deg,
    double time1,
    double time2,
    double gain,
    double lpf_gain);
  bool send_text_command(const std::string & command_text);
  bool send_shutdown_sequence(bool halt_first);

  bool is_connected() const;

private:
  bool open_socket(const std::string & robot_ip, int port, int & sock_fd);
  void start_threads();
  void stop_threads();

  void request_data_loop();
  void read_data_loop();
  void read_cmd_loop();

  bool recv_exact(int sock_fd, void * buffer, std::size_t length);
  bool recv_byte(int sock_fd, unsigned char & value);
  bool send_all(int sock_fd, const void * buffer, std::size_t length);
  bool parse_state_payload(const std::string & payload, Rb10SystemState & state) const;
  void set_disconnected();

  std::string robot_ip_;
  double data_request_rate_hz_{100.0};
  bool realtime_threads_enabled_{false};
  int realtime_priority_{0};
  int cmd_sock_{-1};
  int data_sock_{-1};
  std::atomic<bool> running_{false};
  std::atomic<bool> connected_{false};
  std::mutex command_mutex_;
  std::mutex command_ack_mutex_;
  std::condition_variable command_ack_cv_;
  bool servo_command_response_pending_{false};

  StateCallback state_callback_;
  LogCallback log_callback_;

  std::thread data_request_thread_;
  std::thread data_read_thread_;
  std::thread cmd_read_thread_;
  mutable std::atomic<bool> warned_invalid_self_collision_word_{false};
};

}  // namespace rb10_rmpflow_rviz
