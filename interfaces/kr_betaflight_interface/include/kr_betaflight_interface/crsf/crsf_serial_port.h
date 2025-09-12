#pragma once

#include <atomic>
#include <thread>

#include "kr_betaflight_interface/crsf/crsf_msg.h"

namespace crsf_bridge {

class CrsfSerialPort {
 public:
  CrsfSerialPort();
  CrsfSerialPort(const std::string& port,
                 const rclcpp::Clock::SharedPtr& clock = nullptr);
  virtual ~CrsfSerialPort();

public:
  bool setUpCrsfSerialPort(const std::string& port,
                           const rclcpp::Clock::SharedPtr& clock = nullptr);

  bool connectSerialPort(const std::string& port);
  void disconnectSerialPort();

  bool startReceiverThread();
  bool stopReceiverThread();

  void transmitSerialCrsfMessage(const CrsfMsg& crsf_msg) const;
  virtual void handleReceivedCrsfMessage(
      const crsf_bridge::CrsfMsg& received_crsf_msg) {}

 private:
  static constexpr int kCrsfFrameLength_ = 24; // CRSF frame length (update as needed)
  static constexpr uint8_t kCrsfHeaderByte_ = 0xC8; // CRSF header (update as needed)
  static constexpr int kPollTimeoutMilliSeconds_ = 500;

  bool configureSerialPortForCrsf() const;
  void serialPortReceiveThread();
  crsf_bridge::CrsfMsg parseCrsfMessage(const uint8_t* payload) const;

  std::thread receiver_thread_;
  std::atomic_bool receiver_thread_should_exit_;

  int serial_port_fd_;
  rclcpp::Logger logger_ = rclcpp::get_logger("crsf_serial_port");
  rclcpp::Clock::SharedPtr clock_;
};

}  // namespace crsf_bridge
