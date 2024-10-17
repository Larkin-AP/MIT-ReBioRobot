// imu_receiver.h

#ifndef IMU_RECEIVER_H
#define IMU_RECEIVER_H

#include <thread>
#include <string>

#define FRAME_HEAD 0xfc
#define TYPE_IMU 0x40
#define TYPE_AHRS 0x41
#define TYPE_INSGPS 0x42
#define TYPE_GEODETIC_POS 0x5c
#define TYPE_SYS_STATE 0x50
#define TYPE_BODY_ACCELERATION 0x62
#define TYPE_ACCELERATION 0x61

#define IMU_LEN 56
#define AHRS_LEN 48
#define INSGPS_LEN 72
#define GEODETIC_POS_LEN 32
#define SYS_STATE_LEN 100
#define BODY_ACCELERATION_LEN 16
#define ACCELERATION_LEN 12

class IMUReceiver {
public:
    bool openPort(const std::string& port, unsigned int baud_rate);
    void start();
    void stop();
    void receiveData();
    float imu_data[12];

private:
    bool validateTypeAndLength(char head_type, char check_len);
    int getDataLength(char head_type);

    void processIMUData(const char* data);
    void processAHRSData(const char* data);
    void processINSGPSData(const char* data);

    int serial_fd;
    std::string port_name;
    unsigned int baud_rate;
    int timeout;
    std::thread data_thread;
    bool is_running = false;
};

#endif // IMU_RECEIVER_H
