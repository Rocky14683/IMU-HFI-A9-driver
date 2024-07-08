//
// Created by rockychen on 7/8/24.
//
#include "ros/ros.h"
#include "cmath"
#include "serial/serial.h"
#include "sensor_msgs/Imu.h"

bool checksum(const std::array<char, 45>& list_data, const std::array<char, 2>& check_datas) {
    uint16_t crc = 0xFFFF;
    for(const auto data : list_data) {
        crc ^= data;// 16 bits xor 8 bits
        for(int i = 0; i < 8; i++) {
            if((crc & 1) != 0) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return ((crc & 0xff) << 8) == (check_datas.at(0) << 8 | check_datas.at(1));
}

std::array<float, 10> hex_to_float(const std::array<unsigned char, 40>& raw_data) {

    std::array<float, 10> arr;
    for(int i = 0, idx = 0; i < 40; i+=4, idx++) {
        uint32_t data;
        std::memcpy(&data, &raw_data[i], sizeof(data));

        float value = *reinterpret_cast<float*>(&data);

        arr.at(idx) = value;
    }

    std::reverse(arr.begin(), arr.end());

    return arr;
}


void process_serial_data(char datas) {

}


int main(int argc, char *argv[]) {
    ros::init(argc, argv, "hfi_a9_imu");

    ros::NodeHandle handle;
    serial::Serial imu_serial{};
    try {
        imu_serial = {"/dev/ttyUSB0", 921600, serial::Timeout::simpleTimeout(500)};
        if(!imu_serial.isOpen()) {
            imu_serial.open(); // if cant open it will throw exceptions
        }

        ROS_INFO("Imu serial succefully opened");
    } catch (const std::exception& err) {
        ROS_ERROR(err.what());
        ROS_INFO("Failed to open imu serial port");
        return -1;
    }

    ros::Publisher imu_publisher = handle.advertise<sensor_msgs::Imu>("sensor/imu", 10);
    ros::Rate loop_rate(300);

    while(!ros::isShuttingDown()) {
        size_t buffer_count = imu_serial.available();

        if(buffer_count > 0) {
            auto buffer = imu_serial.read(buffer_count); // char array
            for(const char c : buffer) {
                process_serial_data(c);
            }
        }

        loop_rate.sleep();
    }
    ROS_INFO("Shutting down");
    return 0;
}

