#ifndef CAN_TRANSPORT_HPP
#define CAN_TRANSPORT_HPP
#include <cstdint>
#include <string>
#include "can_protocol.h"

class CanTransport {
public:
    CanTransport() = default;
    ~CanTransport();

    CanTransport(const CanTransport &) = delete;
    CanTransport &operator=(const CanTransport &) = delete;

    bool open(const std::string &ifname);
    void close();

    bool send(uint32_t id, uint8_t dlc, const uint8_t *data);
    bool send(const can_frame_t &frame);

    bool receive(can_frame_t &out, int timeout_ms = 50);

    bool is_open() const { return fd_ >= 0; }

private:
    int fd_ = -1;
};

#endif