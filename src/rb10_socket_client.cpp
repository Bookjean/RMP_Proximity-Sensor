#include "rb10_rmpflow_rviz/rb10_socket_client.hpp"

#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <pthread.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <string>
#include <thread>

namespace rb10_rmpflow_rviz
{

namespace
{

constexpr int kCmdPort = 5000;
constexpr int kDataPort = 5001;
constexpr std::size_t kMaxPacketPayloadSize = 4096;
constexpr std::size_t kRequiredStateWords = 127;

bool is_retryable_errno(int error_number)
{
  return error_number == EAGAIN || error_number == EWOULDBLOCK || error_number == EINTR;
}

std::string trim_newline(const std::string & text)
{
  if (!text.empty() && text.back() == '\n') {
    return text.substr(0, text.size() - 1U);
  }
  return text;
}

void configure_thread_realtime(
  std::thread & thread,
  bool enabled,
  int priority,
  const char * thread_name,
  const Rb10SocketClient::LogCallback & log_callback)
{
  if (!enabled || !thread.joinable()) {
    return;
  }

  sched_param params{};
  params.sched_priority = priority;
  const int rc = ::pthread_setschedparam(thread.native_handle(), SCHED_FIFO, &params);
  if (rc != 0) {
    if (log_callback) {
      log_callback(
        std::string("[socket] failed to switch ") + thread_name +
        " thread to SCHED_FIFO priority " + std::to_string(priority) +
        ": " + std::strerror(rc));
    }
    return;
  }

  if (log_callback) {
    log_callback(
      std::string("[socket] ") + thread_name +
      " thread running with SCHED_FIFO priority " + std::to_string(priority));
  }
}

}  // namespace

Rb10SocketClient::Rb10SocketClient() = default;

Rb10SocketClient::~Rb10SocketClient()
{
  disconnect();
}

bool Rb10SocketClient::connect(
  const std::string & robot_ip,
  double data_request_rate_hz,
  StateCallback state_callback,
  LogCallback log_callback,
  bool enable_realtime_threads,
  int realtime_priority)
{
  disconnect();

  robot_ip_ = robot_ip;
  data_request_rate_hz_ = data_request_rate_hz > 1.0 ? data_request_rate_hz : 100.0;
  state_callback_ = std::move(state_callback);
  log_callback_ = std::move(log_callback);
  realtime_threads_enabled_ = enable_realtime_threads;
  realtime_priority_ = std::max(realtime_priority, 1);

  if (!open_socket(robot_ip_, kCmdPort, cmd_sock_)) {
    set_disconnected();
    return false;
  }
  if (!open_socket(robot_ip_, kDataPort, data_sock_)) {
    set_disconnected();
    return false;
  }

  connected_.store(true);
  running_.store(true);
  start_threads();
  return true;
}

void Rb10SocketClient::disconnect()
{
  stop_threads();
  set_disconnected();
}

bool Rb10SocketClient::initialize_robot(bool simulation_mode)
{
  if (!is_connected()) {
    return false;
  }

  if (!send_text_command("mc jall init")) {
    return false;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(150));

  if (!send_text_command(simulation_mode ? "pgmode simulation" : "pgmode real")) {
    return false;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(150));

  return true;
}

bool Rb10SocketClient::send_movej_degrees(
  const std::array<double, 6> & joint_deg,
  double speed,
  double acceleration)
{
  std::ostringstream command_stream;
  command_stream << std::fixed << std::setprecision(3);
  command_stream << "move_j(jnt[";
  for (std::size_t index = 0; index < joint_deg.size(); ++index) {
    if (index > 0U) {
      command_stream << ",";
    }
    command_stream << joint_deg[index];
  }
  command_stream << "]," << speed << "," << acceleration << ")";
  return send_text_command(command_stream.str());
}

bool Rb10SocketClient::send_servoj_degrees(
  const std::array<double, 6> & joint_deg,
  double time1,
  double time2,
  double gain,
  double lpf_gain)
{
  {
    std::unique_lock<std::mutex> lock(command_ack_mutex_);
    const bool ready_to_send = command_ack_cv_.wait_for(
      lock,
      std::chrono::milliseconds(50),
      [this]() {
        return !servo_command_response_pending_ || !running_.load() || !connected_.load();
      });
    if (!ready_to_send) {
      servo_command_response_pending_ = false;
      if (log_callback_) {
        log_callback_(
          "[socket] timed out waiting for an RB10 command response before sending the next move_servo_j");
      }
      return false;
    }
    if (!is_connected()) {
      return false;
    }
    servo_command_response_pending_ = true;
  }

  std::ostringstream command_stream;
  command_stream << std::fixed << std::setprecision(3);
  command_stream << "move_servo_j(jnt[";
  for (std::size_t index = 0; index < joint_deg.size(); ++index) {
    if (index > 0U) {
      command_stream << ",";
    }
    command_stream << joint_deg[index];
  }
  command_stream << "],";
  command_stream << std::setprecision(6) << time1 << "," << time2 << "," << gain << "," << lpf_gain << ")";
  if (!send_text_command(command_stream.str())) {
    {
      std::scoped_lock lock(command_ack_mutex_);
      servo_command_response_pending_ = false;
    }
    command_ack_cv_.notify_all();
    return false;
  }
  return true;
}

bool Rb10SocketClient::send_text_command(const std::string & command_text)
{
  if (!is_connected()) {
    return false;
  }

  // Match the vendor Python API framing exactly. Sending a trailing newline
  // can trigger parser errors on the RB10 controller UI.
  const std::string payload = command_text + " ";
  std::scoped_lock<std::mutex> lock(command_mutex_);
  if (!send_all(cmd_sock_, payload.data(), payload.size())) {
    set_disconnected();
    return false;
  }
  return true;
}

bool Rb10SocketClient::send_shutdown_sequence(bool halt_first)
{
  const std::string first = halt_first ? "task stop" : "task pause";
  const std::string second = halt_first ? "task pause" : "task stop";

  const bool first_ok = send_text_command(first);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  const bool clear_itpl_ok = send_text_command("move_itpl_clear()");
  const bool clear_pb_ok = send_text_command("move_pb_clear()");
  const bool clear_jb_ok = send_text_command("move_jb_clear()");
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  const bool second_first_ok = send_text_command(first);

  if (first_ok || second_first_ok) {
    return true;
  }

  const bool fallback_ok = send_text_command(second);
  return fallback_ok || clear_itpl_ok || clear_pb_ok || clear_jb_ok;
}

bool Rb10SocketClient::is_connected() const
{
  return connected_.load();
}

bool Rb10SocketClient::open_socket(const std::string & robot_ip, int port, int & sock_fd)
{
  sock_fd = ::socket(AF_INET, SOCK_STREAM, 0);
  if (sock_fd < 0) {
    return false;
  }

  const int enable = 1;
  ::setsockopt(sock_fd, IPPROTO_TCP, TCP_NODELAY, &enable, sizeof(enable));

  timeval timeout{};
  timeout.tv_sec = 0;
  timeout.tv_usec = 100000;
  ::setsockopt(sock_fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
  ::setsockopt(sock_fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_port = htons(static_cast<uint16_t>(port));
  if (::inet_pton(AF_INET, robot_ip.c_str(), &address.sin_addr) != 1) {
    ::close(sock_fd);
    sock_fd = -1;
    return false;
  }

  if (::connect(sock_fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) != 0) {
    ::close(sock_fd);
    sock_fd = -1;
    return false;
  }

  return true;
}

void Rb10SocketClient::start_threads()
{
  data_request_thread_ = std::thread(&Rb10SocketClient::request_data_loop, this);
  data_read_thread_ = std::thread(&Rb10SocketClient::read_data_loop, this);
  cmd_read_thread_ = std::thread(&Rb10SocketClient::read_cmd_loop, this);

  configure_thread_realtime(
    data_request_thread_,
    realtime_threads_enabled_,
    realtime_priority_,
    "data_request",
    log_callback_);
  configure_thread_realtime(
    data_read_thread_,
    realtime_threads_enabled_,
    realtime_priority_,
    "data_read",
    log_callback_);
  configure_thread_realtime(
    cmd_read_thread_,
    realtime_threads_enabled_,
    realtime_priority_,
    "cmd_read",
    log_callback_);
}

void Rb10SocketClient::stop_threads()
{
  running_.store(false);

  if (cmd_sock_ >= 0) {
    ::shutdown(cmd_sock_, SHUT_RDWR);
  }
  if (data_sock_ >= 0) {
    ::shutdown(data_sock_, SHUT_RDWR);
  }

  if (data_request_thread_.joinable()) {
    data_request_thread_.join();
  }
  if (data_read_thread_.joinable()) {
    data_read_thread_.join();
  }
  if (cmd_read_thread_.joinable()) {
    cmd_read_thread_.join();
  }
}

void Rb10SocketClient::request_data_loop()
{
  const auto period = std::chrono::duration<double>(1.0 / std::max(data_request_rate_hz_, 1.0));
  while (running_.load() && connected_.load()) {
    if (!send_all(data_sock_, "reqdata", 7U)) {
      if (log_callback_) {
        log_callback_("[socket] failed to send reqdata on the data socket");
      }
      set_disconnected();
      return;
    }
    std::this_thread::sleep_for(period);
  }
}

void Rb10SocketClient::read_data_loop()
{
  while (running_.load() && connected_.load()) {
    unsigned char start_byte = 0;
    if (!recv_byte(data_sock_, start_byte)) {
      if (running_.load()) {
        if (log_callback_) {
          log_callback_("[socket] data socket closed while waiting for packet start");
        }
        set_disconnected();
      }
      return;
    }

    if (start_byte != 0x24U) {
      continue;
    }

    std::array<unsigned char, 3> header_tail{};
    if (!recv_exact(data_sock_, header_tail.data(), header_tail.size())) {
      if (running_.load()) {
        if (log_callback_) {
          log_callback_("[socket] data socket closed while reading packet header");
        }
        set_disconnected();
      }
      return;
    }

    const std::size_t payload_size =
      static_cast<std::size_t>(header_tail[0]) |
      (static_cast<std::size_t>(header_tail[1]) << 8U);
    const unsigned char packet_type = header_tail[2];

    if (payload_size == 0U || payload_size > kMaxPacketPayloadSize) {
      continue;
    }

    std::string payload(payload_size, '\0');
    if (!recv_exact(data_sock_, payload.data(), payload.size())) {
      if (running_.load()) {
        if (log_callback_) {
          log_callback_("[socket] data socket closed while reading packet payload");
        }
        set_disconnected();
      }
      return;
    }

    if (packet_type != 3U) {
      continue;
    }

    Rb10SystemState state;
    if (!parse_state_payload(payload, state)) {
      continue;
    }

    if (state_callback_) {
      state_callback_(state);
    }
  }
}

void Rb10SocketClient::read_cmd_loop()
{
  std::array<char, 256> buffer{};
  while (running_.load() && connected_.load()) {
    const ssize_t bytes_read = ::recv(cmd_sock_, buffer.data(), buffer.size(), 0);
    if (bytes_read > 0) {
      {
        std::scoped_lock lock(command_ack_mutex_);
        servo_command_response_pending_ = false;
      }
      command_ack_cv_.notify_all();
      if (log_callback_) {
        log_callback_(trim_newline(std::string(buffer.data(), static_cast<std::size_t>(bytes_read))));
      }
      continue;
    }
    if (bytes_read == 0) {
      if (running_.load() && log_callback_) {
        log_callback_(
          "[socket] command socket receive side closed by peer; keeping the session alive until a send fails");
      }
      return;
    }
    if (is_retryable_errno(errno)) {
      continue;
    }
    if (running_.load() && log_callback_) {
      log_callback_(
        std::string("[socket] command socket recv error: ") + std::strerror(errno) +
        "; keeping the session alive until a send fails");
    }
    return;
  }
}

bool Rb10SocketClient::recv_exact(int sock_fd, void * buffer, std::size_t length)
{
  auto * byte_buffer = static_cast<unsigned char *>(buffer);
  std::size_t offset = 0U;
  while (offset < length && running_.load() && connected_.load()) {
    const ssize_t received =
      ::recv(sock_fd, byte_buffer + offset, length - offset, 0);
    if (received > 0) {
      offset += static_cast<std::size_t>(received);
      continue;
    }
    if (received == 0) {
      return false;
    }
    if (is_retryable_errno(errno)) {
      continue;
    }
    return false;
  }
  return offset == length;
}

bool Rb10SocketClient::recv_byte(int sock_fd, unsigned char & value)
{
  while (running_.load() && connected_.load()) {
    const ssize_t received = ::recv(sock_fd, &value, 1, 0);
    if (received == 1) {
      return true;
    }
    if (received == 0) {
      return false;
    }
    if (is_retryable_errno(errno)) {
      continue;
    }
    return false;
  }
  return false;
}

bool Rb10SocketClient::send_all(int sock_fd, const void * buffer, std::size_t length)
{
  const auto * byte_buffer = static_cast<const unsigned char *>(buffer);
  std::size_t offset = 0U;
  while (offset < length && connected_.load()) {
    int send_flags = 0;
#ifdef MSG_NOSIGNAL
    send_flags |= MSG_NOSIGNAL;
#endif
    const ssize_t sent = ::send(sock_fd, byte_buffer + offset, length - offset, send_flags);
    if (sent > 0) {
      offset += static_cast<std::size_t>(sent);
      continue;
    }
    if (sent == 0) {
      return false;
    }
    if (is_retryable_errno(errno)) {
      continue;
    }
    return false;
  }
  return offset == length;
}

bool Rb10SocketClient::parse_state_payload(const std::string & payload, Rb10SystemState & state) const
{
  if (payload.size() < kRequiredStateWords * sizeof(std::uint32_t)) {
    return false;
  }

  const auto read_float = [&payload](std::size_t word_index) {
    float value = 0.0F;
    std::memcpy(&value, payload.data() + word_index * sizeof(std::uint32_t), sizeof(value));
    return value;
  };
  const auto read_int = [&payload](std::size_t word_index) {
    std::int32_t value = 0;
    std::memcpy(&value, payload.data() + word_index * sizeof(std::uint32_t), sizeof(value));
    return value;
  };

  state.controller_time_sec = read_float(0U);
  for (std::size_t index = 0; index < 6U; ++index) {
    state.joint_ref_deg[index] = read_float(1U + index);
    state.joint_ang_deg[index] = read_float(7U + index);
  }
  state.robot_state = read_int(84U);
  state.power_state = read_int(85U);
  state.collision_detect_onoff = read_int(98U);
  state.program_mode = read_int(100U);
  state.init_state_info = read_int(101U);
  state.init_error = read_int(102U);
  state.op_stat_collision_occur = read_int(110U);
  state.op_stat_sos_flag = read_int(111U);
  const std::int32_t raw_self_collision = read_int(112U);
  if (raw_self_collision == 0 || raw_self_collision == 1) {
    state.op_stat_self_collision = raw_self_collision;
  } else {
    state.op_stat_self_collision = 0;
    if (log_callback_ && !warned_invalid_self_collision_word_.exchange(true)) {
      log_callback_(
        std::string(
          "[socket] ignoring non-boolean self_collision word from the RB10 data packet: ") +
        std::to_string(raw_self_collision));
    }
  }
  state.op_stat_soft_estop_occur = read_int(113U);
  state.op_stat_ems_flag = read_int(114U);
  return true;
}

void Rb10SocketClient::set_disconnected()
{
  connected_.store(false);

  if (cmd_sock_ >= 0) {
    ::shutdown(cmd_sock_, SHUT_RDWR);
    ::close(cmd_sock_);
    cmd_sock_ = -1;
  }
  if (data_sock_ >= 0) {
    ::shutdown(data_sock_, SHUT_RDWR);
    ::close(data_sock_);
    data_sock_ = -1;
  }
}

}  // namespace rb10_rmpflow_rviz
