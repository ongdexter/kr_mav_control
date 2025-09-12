#pragma once
#include <cstdint>

class ProtocolMsgBase {
public:
    virtual ~ProtocolMsgBase() = default;
    virtual uint16_t getChannel(int idx) const = 0;
    virtual void setChannel(int idx, uint16_t value) = 0;
};