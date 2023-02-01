#include "CAN.h"
#include <iostream>

ODrive::CAN::CAN() {
    struct sockaddr_can Address;
	struct ifreq IFReq;

    // Socket setup.
    this->SocketFD = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    
    if (this->SocketFD < 0) {
        // Error; throw exception.
        exit(1);
    }

    strcpy(IFReq.ifr_name, "can0"); // Set interface name to can0. Change if needed.
    ioctl(this->SocketFD, SIOCGIFINDEX, &IFReq); // Store interface index in IFReq.

    Address.can_family = AF_CAN;
    Address.can_ifindex = IFReq.ifr_ifindex;

    // Bind address to FD.
    if (bind(this->SocketFD, (struct sockaddr *)&Address, sizeof(Address)) < 0) {
        // Error; throw exception.
        exit(1);
    }
}

ODrive::CAN::~CAN() {
    // Close socket.
    close(this->SocketFD);
}

ssize_t ODrive::CAN::Read(can_frame *Frame) {
    return read(this->SocketFD, Frame, sizeof(can_frame));
}

ssize_t ODrive::CAN::Write(can_frame *Frame) {
    return write(this->SocketFD, Frame, sizeof(can_frame));
}