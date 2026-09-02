/**
 * @file    can_transport.cpp
 * @brief   SocketCAN transport implementation.
 *
 * Provides low-level CAN frame transport for the Jetson application.
 *
 * This layer owns SocketCAN interface management, frame transmission,
 * reception, timeout handling, and socket-level error detection.
 * Application-level CAN message semantics remain in the CAN tasks.
 */

#include "can_transport.hpp"

#include <cstdio>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <poll.h>

CanTransport::~CanTransport()
{
    close();
}

bool CanTransport::open(
    const std::string& ifname)
{
    fd_ =
        socket(
            PF_CAN,
            SOCK_RAW,
            CAN_RAW
        );

    if (fd_ < 0)
    {
        perror(
            "CanTransport: socket()"
        );

        return false;
    }

    /*
     * Enable SocketCAN error-frame reporting so bus and interface errors
     * remain observable by the transport layer.
     */
    can_err_mask_t err_mask =
        CAN_ERR_MASK;

    if (setsockopt(
        fd_,
        SOL_CAN_RAW,
        CAN_RAW_ERR_FILTER,
        &err_mask,
        sizeof(err_mask)) < 0)
    {
        perror(
            "CanTransport: setsockopt(CAN_RAW_ERR_FILTER) - "
            "khong bat duoc error frame reporting"
        );
    }

    struct ifreq ifr;

    std::strncpy(
        ifr.ifr_name,
        ifname.c_str(),
        IFNAMSIZ - 1
    );

    ifr.ifr_name[IFNAMSIZ - 1] =
        '\0';

    if (ioctl(
        fd_,
        SIOCGIFINDEX,
        &ifr) < 0)
    {
        perror(
            "CanTransport: ioctl(SIOCGIFINDEX)"
        );

        ::close(fd_);
        fd_ = -1;

        return false;
    }

    struct sockaddr_can addr;

    std::memset(
        &addr,
        0,
        sizeof(addr)
    );

    addr.can_family =
        AF_CAN;

    addr.can_ifindex =
        ifr.ifr_ifindex;

    if (bind(
        fd_,
        (struct sockaddr*)&addr,
        sizeof(addr)) < 0)
    {
        perror(
            "CanTransport: bind()"
        );

        ::close(fd_);
        fd_ = -1;

        return false;
    }

    /*
     * Increase the kernel receive buffer so short scheduling delays in the
     * CAN RX task do not immediately overflow the socket queue.
     *
     * This is a best-effort optimization; failure does not make the CAN
     * socket unusable.
     */
    {
        int rcvbuf_bytes =
            256 * 1024;

        if (setsockopt(
            fd_,
            SOL_SOCKET,
            SO_RCVBUF,
            &rcvbuf_bytes,
            sizeof(rcvbuf_bytes)) < 0)
        {
            perror(
                "CanTransport: setsockopt(SO_RCVBUF) - "
                "tiep tuc voi buffer mac dinh"
            );
        }
    }

    /*
     * Disable SocketCAN loopback for this transport.
     *
     * The Jetson CAN RX task is intended to process frames received from
     * STM32, not frames transmitted by the Jetson itself. Disabling loopback
     * prevents locally transmitted frames from entering the RX path.
     */
    {
        int loopback_off = 0;

        if (setsockopt(
            fd_,
            SOL_CAN_RAW,
            CAN_RAW_LOOPBACK,
            &loopback_off,
            sizeof(loopback_off)) < 0)
        {
            perror(
                "CanTransport: setsockopt(CAN_RAW_LOOPBACK) - "
                "tiep tuc voi loopback mac dinh"
            );
        }
    }

    return true;
}

void CanTransport::close()
{
    if (fd_ >= 0)
    {
        ::close(fd_);
        fd_ = -1;
    }
}

bool CanTransport::send(
    uint32_t id,
    uint8_t dlc,
    const uint8_t* data)
{
    can_frame_t f;

    f.id = id;
    f.dlc = dlc;

    std::memset(
        f.data,
        0,
        sizeof(f.data)
    );

    if (data && dlc > 0)
    {
        std::memcpy(
            f.data,
            data,
            dlc > 8 ? 8 : dlc
        );
    }

    return send(f);
}

bool CanTransport::send(
    const can_frame_t& frame)
{
    if (fd_ < 0)
    {
        return false;
    }

    struct can_frame raw;

    std::memset(
        &raw,
        0,
        sizeof(raw)
    );

    /*
     * This transport currently uses standard 11-bit CAN identifiers and
     * limits the payload to the classic CAN data field size.
     */
    raw.can_id =
        frame.id & CAN_SFF_MASK;

    raw.can_dlc =
        frame.dlc > 8
        ? 8
        : frame.dlc;

    std::memcpy(
        raw.data,
        frame.data,
        raw.can_dlc
    );

    ssize_t n =
        ::write(
            fd_,
            &raw,
            sizeof(raw)
        );

    return
        n == (ssize_t)sizeof(raw);
}

bool CanTransport::receive(
    can_frame_t& out,
    int timeout_ms)
{
    if (fd_ < 0)
    {
        return false;
    }

    if (timeout_ms >= 0)
    {
        struct pollfd pfd;

        pfd.fd =
            fd_;

        pfd.events =
            POLLIN;

        pfd.revents =
            0;

        int r =
            poll(
                &pfd,
                1,
                timeout_ms
            );

        if (r < 0)
        {
            /*
             * poll() failure is different from a normal timeout.
             *
             * Report the error so an interface or descriptor failure is not
             * silently interpreted as a temporary lack of CAN traffic.
             */
            perror(
                "CanTransport: poll() loi"
            );

            return false;
        }

        if (r == 0)
        {
            /*
             * A timeout is an expected condition when no CAN frame arrives
             * within the requested supervision interval.
             */
            return false;
        }

        /*
         * poll() may report an event without readable CAN data.
         *
         * Treat socket errors and hangups as transport failures instead of
         * proceeding to read() and hiding the original condition.
         */
        if (pfd.revents &
            (POLLERR | POLLHUP | POLLNVAL))
        {
            std::fprintf(
                stderr,
                "CanTransport: socket gap loi "
                "(revents=0x%x) - "
                "co the interface can0 bi down/loi, "
                "can kiem tra lai ket noi/trang thai interface.\n",
                pfd.revents
            );

            return false;
        }

        if (!(pfd.revents & POLLIN))
        {
            return false;
        }
    }

    struct can_frame raw;

    ssize_t n =
        ::read(
            fd_,
            &raw,
            sizeof(raw)
        );

    /*
     * A successful poll does not guarantee that the subsequent read will
     * return a complete CAN frame. Treat any short or failed read as a
     * transport failure.
     */
    if (n != (ssize_t)sizeof(raw))
    {
        if (n < 0)
        {
            perror(
                "CanTransport: read() loi sau khi poll() bao co data"
            );
        }

        return false;
    }

    out.is_error =
        (raw.can_id & CAN_ERR_FLAG) != 0;

    out.err_class =
        raw.can_id & CAN_ERR_MASK;

    out.id =
        raw.can_id & CAN_SFF_MASK;

    out.dlc =
        raw.can_dlc;

    std::memset(
        out.data,
        0,
        sizeof(out.data)
    );

    std::memcpy(
        out.data,
        raw.data,
        raw.can_dlc
    );

    return true;
}