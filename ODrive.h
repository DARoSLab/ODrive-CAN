#ifndef __ODRIVE_H
#define __ODRIVE_H

#include "CAN.h"
#include "ODrive_Typedef.h"
#include <vector>

namespace ODrive {
    class ODrive {
        private:
            CAN *CommChannel;
            int Write(can_frame *Data);
            int Read(can_frame *Data);
            can_frame ConstructCANMessage(uint16_t NodeID, Commands Cmd, uint8_t *Data, uint32_t DataSize);
        public:
            // [De]constructor
            ODrive();
            ~ODrive();

            // Function declarations
            VersionQuery GetVersion(uint16_t Node);
            HeartbeatRequest GetHeartbeat(uint16_t Node);
            void Endstop(uint16_t Node);
            ErrorStatus GetError(uint16_t Node);
            void SetNodeID(uint16_t Node, uint32_t ID);
            void SetState(uint16_t Node, uint32_t State);
            EncoderEstimates GetEncEstimate(uint16_t Node);
            void SetControllerMode(uint16_t Node, ControlMode Ctrl, InputMode Input);
            void SetInputPosition(uint16_t Node, uint32_t Position, uint16_t VelocityFF, uint16_t TorqueFF);
            void SetInputVelocity(uint16_t Node, uint32_t Velocity, uint16_t TorqueFF);
            void SetInputTorque(uint16_t Node, uint32_t Torque);
            void SetLimits(uint16_t Node, uint16_t VelocityLimit, uint16_t CurrentLimit);
            void StartAnticog(uint16_t Node);
            void LimitTrajectoryVel(uint16_t Node, uint32_t Limit);
            void LimitTrajectoryAccel(uint16_t Node, uint32_t AccelLimit, uint32_t DeaccelLimit);
            void SetTrajInertia(uint16_t Node, uint32_t Inertia);
            IQRequest GetIQ(uint16_t Node);
            TemperatureRequest GetTemp(uint16_t Node);
            void Reboot(uint16_t Node);
            PowerRequest GetPower(uint16_t Node);
            void ClearErrors(uint16_t Node);
            void SetAbsPosition(uint16_t Node, uint32_t Position);
            void SetPositionGain(uint16_t Node, uint32_t Gain);
            void SetVelocityGains(uint16_t Node, uint32_t VelGain, uint32_t VelIntegratorGain);
            uint32_t GetADCVoltage(uint16_t Node);
            uint32_t GetControllerError(uint16_t Node);
            void DFUMode(uint16_t Node);
    };
};

#endif