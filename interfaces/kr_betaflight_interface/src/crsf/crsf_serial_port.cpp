#include "kr_betaflight_interface/crsf/crsf_serial_port.h"

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
                               const rclcpp::Clock::SharedPtr& clock)
    : receiver_thread_(),
      receiver_thread_should_exit_(false),
      serial_port_fd_(-1),
      clock_(clock) {
  logger_ = rclcpp::get_logger("crsf_serial_port");
}

CrsfSerialPort::~CrsfSerialPort() { disconnectSerialPort(); }

bool CrsfSerialPort::setUpCrsfSerialPort(const std::string& port,
                                         const rclcpp::Clock::SharedPtr& clock) {
  clock_ = clock;
  if (!connectSerialPort(port)) {
    return false;
  }

  if (!startReceiverThread()) {
    return false;
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
  RCLCPP_INFO(logger_, "Setting CRSF serial port baud rate to %d", (int)spd);

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
    
    // Proper bit packing for 11-bit channels
    uint32_t bit_pos = 0;
    for (int ch = 0; ch < 16; ++ch) {
        uint16_t val = chans[ch] & 0x07FF;  // 11-bit mask
        
        // Calculate byte and bit positions
        uint32_t byte_pos = bit_pos / 8;
        uint32_t bit_offset = bit_pos % 8;
        
        // Write the 11-bit value
        if (bit_offset <= 5) {
            // Value fits in current byte and part of next byte
            payload[byte_pos] |= (val << bit_offset) & 0xFF;
            payload[byte_pos + 1] |= (val >> (8 - bit_offset)) & 0xFF;
        } else {
            // Value spans three bytes
            payload[byte_pos] |= (val << bit_offset) & 0xFF;
            payload[byte_pos + 1] |= (val >> (8 - bit_offset)) & 0xFF;
            payload[byte_pos + 2] |= (val >> (16 - bit_offset)) & 0xFF;
        }
        
        bit_pos += 11;
    }

    // CRC over [type, payload] (not address + length)
    frame[CRSF_RC_CHANNELS_FRAME_SIZE - 1] = crsf_crc8(&frame[2], CRSF_RC_CHANNELS_FRAME_SIZE - 3);

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

                // RCLCPP_INFO(logger_, "Read %zd bytes from CRSF serial port", nread);

                // // print buffer contents for debugging
                // std::string buf_str;
                // for (const auto& byte : bytes_buf) {
                //     char byte_str[4];
                //     snprintf(byte_str, sizeof(byte_str), "%02X ", byte);
                //     buf_str += byte_str;
                // }
                // RCLCPP_INFO(logger_, "Buffer contents: %s", buf_str.c_str());
                
                // Try to extract CRSF frames with proper synchronization
                while (bytes_buf.size() >= 3) {  // Need at least address + length + type
                    // Look for any valid CRSF address (0xC8 or 0xEE)
                    if (bytes_buf[0] == 0xC8 || bytes_buf[0] == 0xEE) {
                        uint8_t frame_length = bytes_buf[1];
                        uint8_t total_frame_size = frame_length + 2;  // +2 for address and length bytes

                        // Validate frame length and ensure we have the complete frame
                        if (frame_length >= 2 && frame_length <= 64) {
                            if (bytes_buf.size() < total_frame_size) {
                                // Not enough data yet, break and wait for next read
                                break;
                            }
                            // Check if this is an RC channels frame
                            if (bytes_buf[2] == CRSF_TYPE_RC_CHANNELS_PACKED) {
                                // Extract the complete frame
                                uint8_t frame[total_frame_size];
                                for (size_t i = 0; i < total_frame_size; ++i) {
                                    frame[i] = bytes_buf[i];
                                }
                                
                                // Check CRC - only process frames with valid CRC
                                uint8_t crc = crsf_crc8(&frame[2], frame_length - 1);  // CRC over type + payload only
                                if (crc == frame[frame_length + 1]) {
                                    // Valid frame - parse and process
                                    CrsfMsg msg = parseCrsfMessage(&frame[3]);  // Skip address, length, type
                                    if (clock_) msg.timestamp = clock_->now();
                                    else msg.timestamp = rclcpp::Clock().now();

                                    if (message_callback_) {
                                        message_callback_(msg);
                                    }
                                } else {
                                    // CRC mismatch - log and discard
                                    RCLCPP_WARN(logger_, "CRSF frame CRC mismatch - discarding frame");
                                }
                                
                                // Remove the processed frame (regardless of CRC result)
                                for (size_t i = 0; i < total_frame_size; ++i) {
                                    bytes_buf.pop_front();
                                }
                                continue;
                            } else {
                                // Not an RC channels frame - remove and continue
                                for (size_t i = 0; i < total_frame_size; ++i) {
                                    bytes_buf.pop_front();
                                }
                                continue;
                            }
                        } else {
                            // Invalid length, pop header and continue searching
                            bytes_buf.pop_front();
                            continue;
                        }
                    }
                    // Not a valid frame start, pop one byte and continue searching
                    bytes_buf.pop_front();
                }
                
                // Prevent buffer from growing too large
                if (bytes_buf.size() > 256) {
                    RCLCPP_WARN(logger_, "Buffer overflow, clearing buffer");
                    bytes_buf.clear();
                }
            }
        }
    }
}

void CrsfSerialPort::setMessageCallback(std::function<void(const CrsfMsg&)> cb) {
    message_callback_ = cb;
}

CrsfMsg CrsfSerialPort::parseCrsfMessage(const uint8_t* payload) const {
    CrsfMsg msg;
    const unsigned numOfChannels = 16;
    const unsigned srcBits = 11;
    const unsigned inputChannelMask = (1 << srcBits) - 1;

    // Initialize all channels to safe default values (center position)
    for (uint8_t n = 0; n < numOfChannels; n++) {
        msg.channels[n] = 1024;  // Center position (2048/2)
    }

    uint8_t bitsMerged = 0;
    uint32_t readValue = 0;
    unsigned readByteIndex = 0;
    
    for (uint8_t n = 0; n < numOfChannels; n++) {
        while (bitsMerged < srcBits) {
            if (readByteIndex >= CRSF_RC_CHANNELS_PAYLOAD_SIZE) {
                RCLCPP_ERROR(logger_, "Payload buffer overflow during channel parsing");
                return msg;  // Return safe default values
            }
            uint8_t readByte = payload[readByteIndex++];
            readValue |= ((uint32_t) readByte) << bitsMerged;
            bitsMerged += 8;
        }
        
        uint16_t channelValue = (readValue & inputChannelMask);
        
        // Validate channel value is within reasonable bounds
        if (channelValue <= 2047) {  // 11-bit max value
            msg.channels[n] = channelValue;
        } else {
            RCLCPP_WARN(logger_, "Invalid channel %d value: %d, using default", n, channelValue);
            msg.channels[n] = 1024;  // Center position
        }
        
        readValue >>= srcBits;
        bitsMerged -= srcBits;
    }
    
    // No digital channels, frame_lost, or failsafe in CRSF RC frame
    msg.digital_channel_1 = false;
    msg.digital_channel_2 = false;
    msg.frame_lost = false;
    msg.failsafe = false;
    
    return msg;
}

}  // namespace crsf_bridge
