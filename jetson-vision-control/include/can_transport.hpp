#ifndef CAN_TRANSPORT_HPP
#define CAN_TRANSPORT_HPP

#include <cstdint>
#include <string>

#include "can_protocol.h"

/*
 * SocketCAN transport wrapper.
 *
 * This class owns the CAN socket used by the Jetson communication tasks.
 * It provides a small interface for opening the CAN interface, transmitting
 * frames, receiving frames, and checking the current socket state.
 *
 * The CAN frame format is defined by can_protocol.h.
 */
class CanTransport
{
public:
    CanTransport() = default;

    ~CanTransport();

    /*
     * Non-copyable because the underlying CAN socket is an owned resource.
     */
    CanTransport(
        const CanTransport&) = delete;

    CanTransport& operator=(
        const CanTransport&) = delete;

    /*
     * Opens the specified SocketCAN interface.
     *
     * Returns true when the interface is successfully initialized and ready
     * for CAN communication.
     */
    bool open(
        const std::string& ifname);

    /*
     * Closes the CAN socket and releases the associated system resource.
     */
    void close();

    /*
     * Sends a CAN frame using the supplied identifier, DLC, and payload.
     *
     * Returns true when the frame is successfully submitted to the CAN
     * interface.
     */
    bool send(
        uint32_t id,
        uint8_t dlc,
        const uint8_t* data);

    /*
     * Sends a complete CAN frame.
     */
    bool send(
        const can_frame_t& frame);

    /*
     * Waits for an incoming CAN frame for up to timeout_ms milliseconds.
     *
     * Returns true when a frame is received successfully.
     */
    bool receive(
        can_frame_t& out,
        int timeout_ms = 50);

    /*
     * Returns true when the CAN socket is currently open.
     */
    bool is_open() const
    {
        return fd_ >= 0;
    }

private:
    int fd_ = -1;
};

#endif