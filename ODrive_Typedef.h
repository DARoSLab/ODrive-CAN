#ifndef __ODRIVE_TYPEDEF_H
#define __ODRIVE_TYPEDEF_H

#include <cstdint>

typedef enum {
    NONE                      = 0x00000000,
    INITIALIZING              = 0x00000001,
    SYSTEM_LEVEL              = 0x00000002,
    TIMING_ERROR              = 0x00000004,
    MISSING_ESTIMATE          = 0x00000008,
    BAD_CONFIG                = 0x00000010,
    DRV_FAULT                 = 0x00000020,
    DC_BUS_OVER_VOLTAGE       = 0x00000100,
    DC_BUS_UNDER_VOLTAGE      = 0x00000200,
    DC_BUS_OVER_CURRENT       = 0x00000400,
    DC_BUS_OVER_REGEN_CURRENT = 0x00000800,
    CURRENT_LIMIT_VIOLATION   = 0x00001000,
    MOTOR_OVER_TEMP           = 0x00002000,
    INVERTER_OVER_TEMP        = 0x00004000,
    VELOCITY_LIMIT_VIOLATION  = 0x00008000,
    POSITION_LIMIT_VIOLATION  = 0x00010000,
    WATCHDOG_TIMER_EXPIRED    = 0x01000000,
    ESTOP_REQUESTED           = 0x02000000,
    SPINOUT_DETECTED          = 0x04000000,
    OTHER_DEVICE_FAILED       = 0x08000000,
} Error;

typedef enum {
    UNDEFINED,                        
    IDLE,                             
    STARTUP_SEQUENCE,                 
    FULL_CALIBRATION_SEQUENCE,        
    MOTOR_CALIBRATION,                
    ENCODER_INDEX_SEARCH,             
    ENCODER_OFFSET_CALIBRATION,       
    CLOSED_LOOP_CONTROL,              
    LOCKIN_SPIN,                      
    ENCODER_DIR_FIND,                  
    HOMING,                            
    ENCODER_HALL_POLARITY_CALIBRATION, 
    ENCODER_HALL_PHASE_CALIBRATION,    
} AxisState;

typedef enum {
    VOLTAGE_CONTROL,
    TORQUE_CONTROL,
    VELOCITY_CONTROL,
    POSITION_CONTROL
} ControlMode;

typedef enum {
    INACTIVE,  
    PASSTHROUGH, 
    VEL_RAMP,    
    POS_FILTER,  
    MIX_CHANNELS,
    TRAP_TRAJ,   
    TORQUE_RAMP, 
    MIRROR,      
    TUNING,      
} InputMode;

typedef enum {
    GetVersion,
    Heartbeat,
    Endstop,
    GetError,
    SetAxisNodeID = 0x6,
    SetAxisState,
    GetEncoderEstimates = 0x9,
    SetControllerMode = 0xb,
    SetInputPosition,
    SetInputVelocity,
    SetInputTorque,
    SetLimits,
    StartAnticogging,
    SetTrajectoryVelocityLimit,
    SetTrajectoryAccelerationLimit,
    SetTrajectoryInertia,
    GetIQ,
    GetTemperature,
    Reboot,
    GetBusVoltageCurrent,
    ClearErrors,
    SetAbsolutePosition,
    SetPositionGain,
    SetVelocityGains,
    GetADCVoltage,
    GetControllerError,
    DFU,
} Commands;

typedef struct {
    uint8_t Protocol;
    uint8_t HW_Major;
    uint8_t HW_Minor;
    uint8_t HW_Variant;
    uint8_t FW_Major;
    uint8_t FW_Minor;
    uint8_t FW_Revision;
    uint8_t FW_Unreleased;
} VersionQuery;

typedef struct {
    uint32_t AxisError;
    uint8_t AxisState;
    uint8_t ProcedureResult;
    bool TrajectoryDone;
} HeartbeatRequest;

typedef struct {
    uint32_t Active;
    uint32_t Reason;
} ErrorStatus;

typedef struct {
    uint32_t Position;
    uint32_t Velocity;
} EncoderEstimates;

typedef struct {
    uint32_t SetPoint;
    uint32_t Measured;
} IQRequest;

typedef struct {
    uint32_t SetPoint;
    uint32_t Measured;
} TemperatureRequest;

typedef struct {
    uint32_t Voltage;
    uint32_t Current;
} PowerRequest;

#endif
