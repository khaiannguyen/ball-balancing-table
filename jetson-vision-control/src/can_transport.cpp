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

CanTransport::~CanTransport() {
    close();
}

bool CanTransport::open(const std::string &ifname) {

    can_err_mask_t err_mask = CAN_ERR_MASK; // nhan TAT CA loai loi
    if (setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_ERR_FILTER,
                    &err_mask, sizeof(err_mask)) < 0) {
        perror("CanTransport: setsockopt(CAN_RAW_ERR_FILTER) - "
                "khong bat duoc error frame reporting");
    }
    
    fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd_ < 0) {
        perror("CanTransport: socket()");
        return false;
    }

    struct ifreq ifr;
    std::strncpy(ifr.ifr_name, ifname.c_str(), IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';
    if (ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) {
        perror("CanTransport: ioctl(SIOCGIFINDEX)");
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(fd_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("CanTransport: bind()");
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    // THEM: tang buffer nhan cua socket. Mac dinh cua kernel co the qua
    // nho — neu thread TaskCanRx bi tre lich (vd do TaskControlLoop chay
    // SCHED_FIFO priority cao chiem CPU), frame CAN den don vao buffer
    // socket; buffer day se bi kernel DROP AM THAM (khop voi so lieu
    // "RX dropped" quan sat duoc trong `ip -statistics link show can0`),
    // gay ra khoang lang du lieu du bus vat ly van hoat dong binh thuong.
    // Khong kiem tra loi setsockopt nghiem trong — day la toi uu "best
    // effort", khong phai dieu kien bat buoc de mo socket thanh cong.
    {
        int rcvbuf_bytes = 256 * 1024; // 256KB, du du cho vai nghin frame
        if (setsockopt(fd_, SOL_SOCKET, SO_RCVBUF, &rcvbuf_bytes, sizeof(rcvbuf_bytes)) < 0) {
            perror("CanTransport: setsockopt(SO_RCVBUF) - tiep tuc voi buffer mac dinh");
        }
    }

    // THEM: tat CAN_RAW_LOOPBACK. Mac dinh SocketCAN BAT loopback — nghia
    // la frame do CHINH Jetson gui di (qua socket TaskCanTx mo tren cung
    // interface can0) se duoc loopback ve MOI socket RAW dang mo tren
    // can0, bao gom ca socket cua TaskCanRx. Dieu nay khien TaskCanRx phai
    // xu ly them ca cac frame Jetson tu gui (0x200/0x201/0x202/0x204/0x2FF,
    // 5 frame/chu ky o 100Hz = 500 frame/s thua), gop phan tang tai khong
    // can thiet, gian tiep lam tang nguy co tre lich/tran buffer o Van de
    // tren. Tat loopback de TaskCanRx CHI nhan frame THAT tu STM32.
    {
        int loopback_off = 0;
        if (setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback_off, sizeof(loopback_off)) < 0) {
            perror("CanTransport: setsockopt(CAN_RAW_LOOPBACK) - tiep tuc voi loopback mac dinh");
        }
    }

    return true;
}

void CanTransport::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

bool CanTransport::send(uint32_t id, uint8_t dlc, const uint8_t *data) {
    can_frame_t f;
    f.id = id;
    f.dlc = dlc;
    std::memset(f.data, 0, sizeof(f.data));
    if (data && dlc > 0) std::memcpy(f.data, data, dlc > 8 ? 8 : dlc);
    return send(f);
}

bool CanTransport::send(const can_frame_t &frame) {
    if (fd_ < 0) return false;

    struct can_frame raw;
    std::memset(&raw, 0, sizeof(raw));
    raw.can_id = frame.id & CAN_SFF_MASK;
    raw.can_dlc = frame.dlc > 8 ? 8 : frame.dlc;
    std::memcpy(raw.data, frame.data, raw.can_dlc);

    ssize_t n = ::write(fd_, &raw, sizeof(raw));
    return n == (ssize_t)sizeof(raw);
}

bool CanTransport::receive(can_frame_t &out, int timeout_ms) {
    if (fd_ < 0) return false;

    if (timeout_ms >= 0) {
        struct pollfd pfd;
        pfd.fd = fd_;
        pfd.events = POLLIN;
        pfd.revents = 0;
        int r = poll(&pfd, 1, timeout_ms);

        if (r < 0) {
            // THEM: loi that su tu poll() (vd EINTR, EBADF) — log ra de
            // phan biet voi truong hop "khong co data" (r == 0, binh
            // thuong khi timeout, KHONG log de tranh spam).
            perror("CanTransport: poll() loi");
            return false;
        }
        if (r == 0) {
            return false; // timeout binh thuong, khong co data - KHONG loi
        }

        // THEM: r > 0 nhung PHAI kiem tra revents THAT SU la POLLIN, vi
        // poll() co the tra ve r>0 do POLLERR/POLLHUP/POLLNVAL (socket
        // gap loi/bi dong) ma KHONG co data thuc su de doc. Code cu bo
        // qua buoc nay, coi r>0 la "co the doc duoc", co the che giau
        // 1 socket da hong ma khong ai biet (di vao read() roi fail am
        // tham, khong log ro nguyen nhan goc).
        if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
            std::fprintf(stderr,
                          "CanTransport: socket gap loi (revents=0x%x) - "
                          "co the interface can0 bi down/loi, can kiem tra "
                          "lai ket noi/trang thai interface.\n", pfd.revents);
            return false;
        }
        if (!(pfd.revents & POLLIN)) {
            return false; // khong phai POLLIN, khong co gi de doc
        }
    }

    struct can_frame raw;
    ssize_t n = ::read(fd_, &raw, sizeof(raw));
    if (n != (ssize_t)sizeof(raw)) {
        // THEM: log khi read() that bai du poll() bao co data — day la
        // dau hieu bat thuong (vd socket bi dong giua chung boi loi khac),
        // dang duoc quan tam thay vi am tham tra false nhu code cu.
        if (n < 0) {
            perror("CanTransport: read() loi sau khi poll() bao co data");
        }
        return false;
    }

    out.id = raw.can_id & CAN_SFF_MASK;
    out.dlc = raw.can_dlc;
    std::memset(out.data, 0, sizeof(out.data));
    std::memcpy(out.data, raw.data, raw.can_dlc);
    return true;
}