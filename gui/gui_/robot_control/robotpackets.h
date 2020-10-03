#ifndef ROBOTPACKETS_H
#define ROBOTPACKETS_H
#include <cstdlib>
#include <stdint.h>
//necessary pragma which removes ZERO bytes
//See Servosila documentation for more details
#pragma pack(push,1)
struct PacketConsts
{
    static const int TELEMETRY_PACKET_SIZE = 275;
    static const int TELEMETRY_PACKET_ID = 2;
    static const int REMOTE_CONTROL_PACKET_SIZE = 57;
    static const int REMOTE_CONTROL_PACKET_ID = 1;
    static const int VIDEO_FRAME_PACKET_ID = 4;
};
struct RemoteControlPacket{
    uint8_t FRAME_TYPE_ID=1;
    int16_t AXIS[16];
    uint8_t BUTTON[16];
    double TELEMETRY;
};
struct MOTOR_DATA{
    uint8_t DEVICE_ID;
    uint8_t DEVICE_STATE;
    uint8_t OPERATION_MODE;
    uint32_t POSITION;
    int16_t SPEED;
    int16_t AMPS;
    int16_t STATUS_BITS;
    uint32_t POSITION_COMMAND;
    int16_t SPEED_COMMAND;
    int16_t AMPS_COMMAND;
    uint8_t FAULT_DETECTED;
    uint16_t FAULTS_COUNTER;

};
struct TelemetryPacket{
    uint8_t FRAME_TYPE_ID = 2;
    uint64_t TICK_NUMBER;
    uint8_t NUMBER_OF_MOTORS;
    MOTOR_DATA M_DATA[10];
    int8_t BLANK_BYTES[25];

};

struct VideoFramePacket{
    uint8_t FRAME_TYPE_ID = 4;
    //TODO: libjpeg library to handle jpeg image
};
struct ServoControlPacket{

};

//#pragma pack(pop)

#endif // ROBOTPACKETS_H
