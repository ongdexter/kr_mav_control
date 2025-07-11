#pragma once

#include <atomic>
#include <thread>

#include "kr_sbus_interface/sbus_msg.h"

namespace sbus_bridge {

class SBusSerialPort {
 public:
  SBusSerialPort();
  SBusSerialPort(const std::string& port,
                 const bool start_receiver_thread,
                 const rclcpp::Clock::SharedPtr& clock = nullptr);
  virtual ~SBusSerialPort();

 protected:
  bool setUpSBusSerialPort(const std::string& port,
                           const bool start_receiver_thread,
                           const rclcpp::Clock::SharedPtr& clock = nullptr);

  bool connectSerialPort(const std::string& port);
  void disconnectSerialPort();

  bool startReceiverThread();
  bool stopReceiverThread();

  void transmitSerialSBusMessage(const SBusMsg& sbus_msg) const;
  virtual void handleReceivedSbusMessage(
      const sbus_bridge::SBusMsg& received_sbus_msg) = 0;

 private:
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
