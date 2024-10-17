// imu_receiver.cpp

#include "imu_receiver.h"
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <thread>
#include <chrono>
#include <time.h>

using namespace std;


bool IMUReceiver::openPort(const std::string& port, unsigned int baud_rate) {
    serial_fd = -1; 
    port_name = port;
    baud_rate = baud_rate; 
    timeout = 10;
    serial_fd = open(port_name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_fd == -1) {
        // cerr << "Error opening port: " << port_name << endl;
        std::cerr << "bbb Error opening port " << port << ": " << strerror(errno) << std::endl;
        return false;
    }

    fcntl(serial_fd, F_SETFL, 0);  // Clear all flags on the descriptor

    struct termios options;
    tcgetattr(serial_fd, &options);

    // Set baud rate
    speed_t baud;
    switch (baud_rate) {
        case 921600: baud = B921600; break;
        case 115200: baud = B115200; break;
        default: cerr << "Unsupported baud rate" << endl; return false;
    }
    cfsetispeed(&options, baud);
    cfsetospeed(&options, baud);

    // Set data bits, parity, stop bits
    options.c_cflag |= (CLOCAL | CREAD);    // Enable receiver and set local mode
    options.c_cflag &= ~PARENB;             // No parity
    options.c_cflag &= ~CSTOPB;             // 1 stop bit
    options.c_cflag &= ~CSIZE;              // Clear current data bit settings
    options.c_cflag |= CS8;                 // 8 data bits

    // Set the timeout (in deciseconds)
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = timeout * 10;  // Timeout in tenths of a second

    tcsetattr(serial_fd, TCSANOW, &options);  // Apply settings

    return true;
}

void IMUReceiver::receiveData() {
    if (serial_fd == -1) {
        cerr << "Serial port is not opened!" << endl;
        return;
    }
    printf("Test receive bbb!!!\n");

    char buffer[256];
    while (true) {
        // Read frame head
        if (read(serial_fd, buffer, 1) <= 0) continue;
        if (buffer[0] != (char)FRAME_HEAD) continue;

        // Read data type
        if (read(serial_fd, buffer, 1) <= 0) continue;
        char head_type = buffer[0];

        // Validate data length
        if (read(serial_fd, buffer, 1) <= 0) continue;
        char check_len = buffer[0];
        if (!validateTypeAndLength(head_type, check_len)) continue;

        // Read the data packet
        int data_len = getDataLength(head_type);
        if (read(serial_fd, buffer, data_len) <= 0) continue;
        //std::cout << "head_type" << head_type << std::endl;

        // Process data based on the type
        if (head_type == TYPE_IMU) {
            processIMUData(buffer);
        } else if (head_type == TYPE_AHRS) {
            processAHRSData(buffer);
        } else if (head_type == TYPE_INSGPS) {
            processINSGPSData(buffer);
        }
    }
}

bool IMUReceiver::validateTypeAndLength(char head_type, char check_len) {
    switch (head_type) {
        case TYPE_IMU: return check_len == IMU_LEN;
        case TYPE_AHRS: return check_len == AHRS_LEN;
        case TYPE_INSGPS: return check_len == INSGPS_LEN;
        case TYPE_GEODETIC_POS: return check_len == GEODETIC_POS_LEN;
        case TYPE_SYS_STATE: return check_len == SYS_STATE_LEN;
        case TYPE_BODY_ACCELERATION: return check_len == BODY_ACCELERATION_LEN;
        case TYPE_ACCELERATION: return check_len == ACCELERATION_LEN;
        default: return false;
    }
}

void IMUReceiver::processIMUData(const char* data) {
    memcpy(this->imu_data, data, 12 * sizeof(float));
    //cout << "IMU Data: Gyro_X: " << imu_data[0] << ", Gyro_Y: " << imu_data[1] << ", Gyro_Z: " << imu_data[2] << endl;
}

void IMUReceiver::processAHRSData(const char* data) {
    float ahrs_data[10];
    memcpy(ahrs_data, data, 10 * sizeof(float));
    //cout << "AHRS Data: Roll: " << ahrs_data[3] << ", Pitch: " << ahrs_data[4] << ", Heading: " << ahrs_data[5] << endl;
}

void IMUReceiver::processINSGPSData(const char* data) {
    float insgps_data[16];
    memcpy(insgps_data, data, 16 * sizeof(float));
    cout << "INSGPS Data: Velocity_X: " << insgps_data[0] << ", Velocity_Y: " << insgps_data[1] << endl;
}

int IMUReceiver::getDataLength(char head_type) {
    switch (head_type) {
        case TYPE_IMU: return IMU_LEN;
        case TYPE_AHRS: return AHRS_LEN;
        case TYPE_INSGPS: return INSGPS_LEN;
        case TYPE_GEODETIC_POS: return GEODETIC_POS_LEN;
        case TYPE_SYS_STATE: return SYS_STATE_LEN;
        case TYPE_BODY_ACCELERATION: return BODY_ACCELERATION_LEN;
        case TYPE_ACCELERATION: return ACCELERATION_LEN;
        default: return 0;
    }
}

void IMUReceiver::start() {
    is_running = true;
    data_thread = std::thread(&IMUReceiver::receiveData, this);
    data_thread.detach();
}

void IMUReceiver::stop() {
    is_running = false;
    if (data_thread.joinable()) data_thread.join();
}
