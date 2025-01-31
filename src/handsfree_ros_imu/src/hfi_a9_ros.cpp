//
// Created by rockychen on 7/8/24.
//
#include <numeric>
#include <memory>
#include <serial/serial.h>

#include "ros/ros.h"
#include "cmath"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/transform_datatypes.h"
#include "tf/transform_datatypes.h"

#define G (9.8)

namespace global {
    static bool enable_gra_normalize = true;
    std::shared_ptr<ros::Publisher> imu_publisher;
    std::shared_ptr<ros::Publisher> mag_publisher;
};

bool checksum(const std::vector<uint8_t>& list_data, const std::array<uint8_t, 2>& check_data) {
    uint16_t crc = 0xFFFF;

    for (const uint8_t pos : list_data) {
        crc ^= pos;
        for (int i = 0; i < 8; ++i) {
            if (crc & 1) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }

    const uint16_t result = ((crc & 0xFF) << 8) | (crc >> 8);
    const uint16_t check = (check_data[0] << 8) | check_data[1];

    return result == check;
}

std::vector<float> hex_to_float(std::vector<uint8_t> raw_data) {

    std::vector<float> vec;

    std::reverse(raw_data.begin(), raw_data.end());
    for(int i = 0; i < raw_data.size(); i+=4) {
        uint32_t data;
        std::memcpy(&data, &raw_data[i], 4);

        float value = *reinterpret_cast<float*>(&data);

        vec.push_back(value);
    }

    std::reverse(vec.begin(), vec.end());

    return vec;
}


static void process_serial_data(uint8_t data) {
    static size_t idx = 0;
    static std::pair<bool, bool> pub_flag = {true, true};
    static size_t data_nack_count = 0;
    static std::vector<uint8_t> buffer{};
    sensor_msgs::Imu imu_msg{};
    sensor_msgs::MagneticField mag_msg{};

    if(data_nack_count > 200000) {
        ROS_ERROR("Data NACK");
        exit(-1);
    }



    buffer.push_back(data);


    // printf("%lu : %x\n", idx, data);

    idx++;
    // start bits[0, 1]: aa 55
    // end bits[last 2 idx]: for checksum
    if(buffer.at(0) != 0xaa) {
        data_nack_count++;
        idx = 0;
        return;
    }
    if(idx < 3) {
        return;
    }
    if(buffer.at(1) != 0x55) {
        idx = 0;
        return;
    }

    // buffer[2] : length could be 20 (hex: 14) or 44 (hex: 2c)
    const size_t data_length = buffer.at(2);
    if(idx < buffer.at(2) + 5) {
        return;
    }
    data_nack_count = 0;
    const std::array<uint8_t, 2> check_sum_datas = {*(buffer.cend() - 2), *(buffer.cend() - 1)};
    const std::vector<uint8_t> datas(buffer.cbegin() + 2, buffer.cend() - 2);

    std::array<float, 3> angular_velocity{};
    std::array<float, 3> acceleration{};
    std::array<float, 3> magnetometer{};

    std::array<float, 3> angle_deg{};

    if(data_length == 44 && pub_flag.first) {
        //buffer size = 49
        if(checksum(datas, check_sum_datas)) {
            const std::vector<float> return_datas = hex_to_float(
                {buffer.cbegin() + 7, buffer.cend() - 2}); //ret size 11
            angular_velocity = {return_datas.at(1), return_datas.at(2), return_datas.at(3)};
            acceleration = {return_datas.at(4), return_datas.at(5), return_datas.at(6)};
            magnetometer = {return_datas.at(7) , return_datas.at(8), return_datas.at(9)};
        } else {
            ROS_WARN("Failed to calibrate");
        }
        pub_flag.first = false;
    } else if (data_length == 20 && pub_flag.second) {
        //buffer size = 25
        if(checksum(datas, check_sum_datas)) {
            //ret size 5
            const std::vector<float> return_datas = hex_to_float(
                {buffer.cbegin() + 7, buffer.cend() - 2});
            angle_deg = {return_datas.at(1), return_datas.at(2), return_datas.at(3)};
        } else {
            ROS_WARN("Failed to calibrate");
        }
        pub_flag.second = false;
    } else {
        ROS_WARN("Invalid datas");
    }

    buffer.clear();
    idx = 0;
    pub_flag.first = pub_flag.second = true;

    const ros::Time time_stamp = ros::Time::now();
    imu_msg.header.stamp = time_stamp;
    imu_msg.header.frame_id = "base_link";

    mag_msg.header.stamp = time_stamp;
    mag_msg.header.frame_id = "base_link";

    std::array<float, 3> angle_radian;

    std::transform(angle_deg.begin(), angle_deg.end(), angle_radian.begin(),
        [](const auto& it) {
            return it * M_PI / 180.0;
        });


    tf2::Quaternion qua;

    qua.setEuler(angle_radian.at(0), -angle_radian.at(1), -angle_radian.at(2));
    qua = qua.normalize(); // quaternion_from_euler

    imu_msg.orientation.x = qua.getX();
    imu_msg.orientation.y = qua.getY();
    imu_msg.orientation.z = qua.getZ();
    imu_msg.orientation.w = qua.getW();

    imu_msg.angular_velocity.x = angular_velocity.at(0);
    imu_msg.angular_velocity.y = angular_velocity.at(1);
    imu_msg.angular_velocity.z = angular_velocity.at(2);

    float acc_k = std::sqrt(std::accumulate(acceleration.cbegin(), acceleration.cend(), 0.0,
    [](const float sum, const float val) -> float {
        return sum + val * val;
    }));

    if(acc_k == 0) acc_k = 1.f;

    if(global::enable_gra_normalize) {
        imu_msg.linear_acceleration.x = acceleration.at(0) * -G / acc_k;
        imu_msg.linear_acceleration.x = acceleration.at(1) * -G / acc_k;
        imu_msg.linear_acceleration.x = acceleration.at(2) * -G / acc_k;
    } else {
        imu_msg.linear_acceleration.x = acceleration.at(0) * -G;
        imu_msg.linear_acceleration.x = acceleration.at(1) * -G;
        imu_msg.linear_acceleration.x = acceleration.at(2) * -G;
    }

    mag_msg.magnetic_field.x = magnetometer.at(0);
    mag_msg.magnetic_field.y = magnetometer.at(1);
    mag_msg.magnetic_field.z = magnetometer.at(2);

    global::imu_publisher->publish(imu_msg);
    global::mag_publisher->publish(mag_msg);

}


int main(int argc, char *argv[]) {
    ros::init(argc, argv, "hfi_a9_imu");

    ros::NodeHandle handle;
    handle.getParam("/handsfree_ros_imu/gra_normalization", global::enable_gra_normalize);

    std::string imu_port_string;
    handle.getParam("/handsfree_ros_imu/imu_port", imu_port_string);


    std::unique_ptr<serial::Serial> imu_serial = nullptr;

    try {
        imu_serial = std::make_unique<serial::Serial>("/dev/ttyUSB0", 921600, serial::Timeout::simpleTimeout(500));

        if(!imu_serial->isOpen()) {
            imu_serial->open(); // if cant open it will throw exceptions
        }

        ROS_INFO("Imu serial succefully opened");
    } catch (const std::exception& err) {
        printf("%s\n", err.what());
        ROS_INFO("Failed to open imu serial port");
        return -1;
    }

    global::imu_publisher = std::make_shared<ros::Publisher>(handle.advertise<sensor_msgs::Imu>("handsfree/imu", 10));
    global::mag_publisher = std::make_shared<ros::Publisher>(handle.advertise<sensor_msgs::MagneticField>("handsfree/mag", 10));
    ros::Rate loop_rate(300);

    try {
        imu_serial->flush();

        while(!ros::isShuttingDown()) {
            const size_t buffer_count = imu_serial->available();

            if(buffer_count > 0) {
                const std::string buffer = imu_serial->read(buffer_count); // char array
                for(const uint8_t c : buffer) {
                    process_serial_data(c); // idk man, i dont want to pass this everytime. But its only 3 bytes so i think its fine
                }
            }

            loop_rate.sleep();
        }
    } catch (const std::exception& e) {
        printf("%s\n", e.what());
        ROS_ERROR("Died : (");
    }

    ROS_INFO("Shutting down");
    return 0;
}

