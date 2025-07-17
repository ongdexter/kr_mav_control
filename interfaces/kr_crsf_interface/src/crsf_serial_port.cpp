#include "kr_crsf_interface/crsf_serial_port.h"

#include <asm/ioctls.h>
#include <asm/termbits.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/poll.h>
#include <deque>
#include <vector>
#include <cstring>

#include <rclcpp/rclcpp.hpp>

namespace crsf_bridge {

namespace {
constexpr uint8_t CRSF_ADDRESS_FC = 0xC8;
constexpr uint8_t CRSF_TYPE_RC_CHANNELS_PACKED = 0x16;
constexpr size_t CRSF_RC_CHANNELS_PAYLOAD_SIZE = 22;
constexpr size_t CRSF_RC_CHANNELS_FRAME_SIZE = 26; // 1(addr)+1(len)+1(type)+22(payload)+1(crc)

// CRC-8/MAXIM (poly 0xD5)
uint8_t crsf_crc8(const uint8_t* data, size_t len) {
    uint8_t crc = 0;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int j = 0; j < 8; ++j) {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0xD5;
            else
                crc <<= 1;
        }
    }
    return crc;
}
}

CrsfSerialPort::CrsfSerialPort()
    : receiver_thread_(),
      receiver_thread_should_exit_(false),
      serial_port_fd_(-1),
      clock_(nullptr) {
  logger_ = rclcpp::get_logger("crsf_serial_port");
}

CrsfSerialPort::CrsfSerialPort(const std::string& port,
                               const bool start_receiver_thread,
                               const rclcpp::Clock::SharedPtr& clock)
    : receiver_thread_(),
      receiver_thread_should_exit_(false),
      serial_port_fd_(-1),
      clock_(clock) {
  logger_ = rclcpp::get_logger("crsf_serial_port");
}

CrsfSerialPort::~CrsfSerialPort() { disconnectSerialPort(); }

bool CrsfSerialPort::setUpCrsfSerialPort(const std::string& port,
                                         const bool start_receiver_thread,
                                         const rclcpp::Clock::SharedPtr& clock) {
  clock_ = clock;
  if (!connectSerialPort(port)) {
    return false;
  }

  if (start_receiver_thread) {
    if (!startReceiverThread()) {
      return false;
    }
  }

  return true;
}

bool CrsfSerialPort::connectSerialPort(const std::string& port) {
  serial_port_fd_ = open(port.c_str(), O_RDWR | O_NOCTTY);

  if (serial_port_fd_ == -1) {
    RCLCPP_ERROR(logger_, "Could not open serial port %s", port.c_str());
    return false;
  }

  if (!configureSerialPortForCrsf()) {
    close(serial_port_fd_);
    RCLCPP_ERROR(logger_, "Could not set necessary configuration of serial port");
    return false;
  }

  RCLCPP_INFO(logger_, "Connected to serial port %s", port.c_str());
  return true;
}

void CrsfSerialPort::disconnectSerialPort() {
  stopReceiverThread();
  if (serial_port_fd_ != -1) {
    close(serial_port_fd_);
    serial_port_fd_ = -1;
  }
}

bool CrsfSerialPort::startReceiverThread() {
  try {
    receiver_thread_ =
        std::thread(&CrsfSerialPort::serialPortReceiveThread, this);
  } catch (...) {
    RCLCPP_ERROR(logger_, "Could not successfully start CRSF receiver thread.");
    return false;
  }

  return true;
}

bool CrsfSerialPort::stopReceiverThread() {
  if (!receiver_thread_.joinable()) {
    return true;
  }

  receiver_thread_should_exit_ = true;
  receiver_thread_.join();
  return true;
}

bool CrsfSerialPort::configureSerialPortForCrsf() const {
  fcntl(serial_port_fd_, F_SETFL, 0);
  fcntl(serial_port_fd_, F_SETFL, FNDELAY);

  struct termios2 uart_config;
  ioctl(serial_port_fd_, TCGETS2, &uart_config);

  uart_config.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OPOST);
  uart_config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);
  uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
  uart_config.c_cflag &= ~(CSIZE | PARODD | CBAUD);
  uart_config.c_cflag |= CS8;
  uart_config.c_cflag |= BOTHER;

  // CRSF baud rate is 420000
  const speed_t spd = 420000;
  uart_config.c_ispeed = spd;
  uart_config.c_ospeed = spd;

  if (ioctl(serial_port_fd_, TCSETS2, &uart_config) < 0) {
    RCLCPP_ERROR(logger_, "Could not set configuration of serial port");
    return false;
  }

  return true;
}

void CrsfSerialPort::transmitSerialCrsfMessage(const CrsfMsg& crsf_msg) const {
    uint8_t frame[CRSF_RC_CHANNELS_FRAME_SIZE] = {0};
    frame[0] = CRSF_ADDRESS_FC;
    frame[1] = CRSF_RC_CHANNELS_PAYLOAD_SIZE + 2; // type + payload + crc
    frame[2] = CRSF_TYPE_RC_CHANNELS_PACKED;

    // Pack 16 channels (11 bits each) into 22 bytes (LSB first)
    uint16_t chans[16];
    for (int i = 0; i < 16; ++i) {
        chans[i] = crsf_msg.channels[i];
    }
    uint8_t* payload = &frame[3];
    memset(payload, 0, CRSF_RC_CHANNELS_PAYLOAD_SIZE);
    uint32_t bit_ofs = 0;
    for (int ch = 0; ch < 16; ++ch) {
        uint16_t val = chans[ch] & 0x07FF;
        int byte_idx = bit_ofs / 8;
        int bit_idx = bit_ofs % 8;
        payload[byte_idx] |= val << bit_idx;
        if (bit_idx > 5) {
            payload[byte_idx + 1] |= val >> (8 - bit_idx);
        }
        if (bit_idx > -3) {
            payload[byte_idx + 2] |= val >> (16 - bit_idx);
        }
        bit_ofs += 11;
    }

    // CRC over [address, length, type, payload]
    frame[CRSF_RC_CHANNELS_FRAME_SIZE - 1] = crsf_crc8(frame, CRSF_RC_CHANNELS_FRAME_SIZE - 1);

    int written = write(serial_port_fd_, frame, CRSF_RC_CHANNELS_FRAME_SIZE);
    if (written != (int)CRSF_RC_CHANNELS_FRAME_SIZE) {
        RCLCPP_ERROR(logger_, "Wrote %d bytes but should have written %zu", written, CRSF_RC_CHANNELS_FRAME_SIZE);
    }
}

void CrsfSerialPort::serialPortReceiveThread() {
    struct pollfd fds[1];
    fds[0].fd = serial_port_fd_;
    fds[0].events = POLLIN;
    std::deque<uint8_t> bytes_buf;
    while (!receiver_thread_should_exit_) {
        uint8_t read_buf[128];
        if (poll(fds, 1, kPollTimeoutMilliSeconds_) > 0) {
            if (fds[0].revents & POLLIN) {
                ssize_t nread = read(serial_port_fd_, read_buf, sizeof(read_buf));
                for (ssize_t i = 0; i < nread; ++i) {
                    bytes_buf.push_back(read_buf[i]);
                }
                // Try to extract CRSF frames
                while (bytes_buf.size() >= CRSF_RC_CHANNELS_FRAME_SIZE) {
                    // Look for header
                    if (bytes_buf[0] == CRSF_ADDRESS_FC &&
                        bytes_buf[2] == CRSF_TYPE_RC_CHANNELS_PACKED &&
                        bytes_buf[1] == CRSF_RC_CHANNELS_PAYLOAD_SIZE + 2) {
                        // Check CRC
                        uint8_t frame[CRSF_RC_CHANNELS_FRAME_SIZE];
                        for (size_t i = 0; i < CRSF_RC_CHANNELS_FRAME_SIZE; ++i) {
                            frame[i] = bytes_buf[i];
                        }
                        uint8_t crc = crsf_crc8(frame, CRSF_RC_CHANNELS_FRAME_SIZE - 1);
                        if (crc == frame[CRSF_RC_CHANNELS_FRAME_SIZE - 1]) {
                            // Valid frame
                            CrsfMsg msg = parseCrsfMessage(&frame[3]);
                            if (clock_) msg.timestamp = clock_->now();
                            else msg.timestamp = rclcpp::Clock().now();
                            handleReceivedCrsfMessage(msg);
                            for (size_t i = 0; i < CRSF_RC_CHANNELS_FRAME_SIZE; ++i) bytes_buf.pop_front();
                            continue;
                        }
                    }
                    // Not a valid frame, pop one byte
                    bytes_buf.pop_front();
                }
            }
        }
    }
}

CrsfMsg CrsfSerialPort::parseCrsfMessage(const uint8_t* payload) const {
    CrsfMsg msg;
    // Unpack 16 channels (11 bits each) from 22 bytes
    uint32_t bit_ofs = 0;
    for (int ch = 0; ch < 16; ++ch) {
        int byte_idx = bit_ofs / 8;
        int bit_idx = bit_ofs % 8;
        uint16_t val = (payload[byte_idx] >> bit_idx) | (payload[byte_idx + 1] << (8 - bit_idx));
        if (bit_idx > 5) {
            val |= (payload[byte_idx + 2] << (16 - bit_idx));
        }
        msg.channels[ch] = val & 0x07FF;
        bit_ofs += 11;
    }
    // No digital channels, frame_lost, or failsafe in CRSF RC frame
    msg.digital_channel_1 = false;
    msg.digital_channel_2 = false;
    msg.frame_lost = false;
    msg.failsafe = false;
    return msg;
}

}  // namespace crsf_bridge
