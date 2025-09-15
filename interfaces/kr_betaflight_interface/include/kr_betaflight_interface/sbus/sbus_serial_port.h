#pragma once

#include <atomic>
#include <thread>
#include "kr_betaflight_interface/sbus/sbus_msg.h"

namespace sbus_bridge {

class SBusSerialPort {
 public:
  SBusSerialPort();
  SBusSerialPort(const std::string& port,
                 const rclcpp::Clock::SharedPtr& clock = nullptr);
  virtual ~SBusSerialPort();

public:
  bool setUpSBusSerialPort(const std::string& port,
                           const rclcpp::Clock::SharedPtr& clock = nullptr);

  bool connectSerialPort(const std::string& port);
  void disconnectSerialPort();

  bool startReceiverThread();
  bool stopReceiverThread();

  void transmitSerialSBusMessage(const SBusMsg& sbus_msg) const;
  void setMessageCallback(std::function<void(const SBusMsg&)> cb);

private:
  std::function<void(const SBusMsg&)> message_callback_;
  static constexpr int kSbusFrameLength_ = 25;
  static constexpr uint8_t kSbusHeaderByte_ = 0x0F;
  static constexpr uint8_t kSbusFooterByte_ = 0x00;
  static constexpr int kPollTimeoutMilliSeconds_ = 500;

  bool configureSerialPortForSBus() const;
  void serialPortReceiveThread();
  sbus_bridge::SBusMsg parseSbusMessage(
  uint8_t sbus_msg_bytes[kSbusFrameLength_]) const;

  std::thread receiver_thread_;
  std::atomic_bool receiver_thread_should_exit_;

  int serial_port_fd_;
  rclcpp::Logger logger_ = rclcpp::get_logger("sbus_serial_port");
  rclcpp::Clock::SharedPtr clock_;
};

}  // namespace sbus_bridge
