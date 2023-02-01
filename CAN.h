#ifndef __CAN_H
#define __CAN_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>

namespace ODrive {
    class CAN {
        private:
            int SocketFD;
        public:
            CAN();
            ~CAN();
            ssize_t Read(can_frame *Frame);
            ssize_t Write(can_frame *Frame);
    };
};

#endif