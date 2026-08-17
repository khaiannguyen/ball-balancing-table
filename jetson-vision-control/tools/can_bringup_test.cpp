#include "can_transport.hpp"
#include <cstdio>
#include <cstring>

int main(int argc, char **argv) {
    if (argc < 2) {
        std::printf("Usage: %s [send|recv]\n", argv[0]);
        return 1;
    }

    CanTransport can;
    if (!can.open("can0")) {
        std::printf("Khong mo duoc can0 - kiem tra da 'ip link set can0 up' chua\n");
        return 1;
    }
    std::printf("Da mo can0 thanh cong.\n");

    if (std::strcmp(argv[1], "send") == 0) {
        uint8_t data[8] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};
        bool ok = can.send(CAN_ID_SERVO_CALIB, 8, data);
        std::printf("Gui 0x203: %s\n", ok ? "OK" : "FAIL");
        return ok ? 0 : 1;
    }

    if (std::strcmp(argv[1], "recv") == 0) {
        std::printf("Dang nhan frame trong 5 giay...\n");
        can_frame_t f;
        for (int i = 0; i < 100; i++) {
            if (can.receive(f, 50)) {
                std::printf("RX id=0x%03X dlc=%d data=", f.id, f.dlc);
                for (int j = 0; j < f.dlc; j++) std::printf("%02X ", f.data[j]);
                std::printf("\n");
            }
        }
        return 0;
    }

    std::printf("Tham so khong hop le: %s\n", argv[1]);
    return 1;
}