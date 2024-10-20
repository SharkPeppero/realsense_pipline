/**
 * @brief Realsense Pipline Stream
 */

#ifndef CAMERA_PRODUCER_CAMERA_REALSENSE_H
#define CAMERA_PRODUCER_CAMERA_REALSENSE_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <librealsense2/hpp/rs_processing.hpp>
#include <librealsense2/hpp/rs_types.hpp>
#include <librealsense2/hpp/rs_sensor.hpp>

/**
 * 功能:
 *  1、设置取流的数据类型
 *      rgb的
 *
 *  2、设置相机的内参属性
 *      设备的序列号
 *      rgb的 分辨率 fps 内参 畸变 相机类型 数据格式
 *      depth的 分辨率 fps 内参 畸变 相机类型 数据格式
 *      imu的 fps
 *      外参关系
 */
class RealsenseD435i {
public:

    struct Options {

        /// RGB Parameter
        bool use_rgb_ = true;
        int rgb_width_ = 640;
        int rgb_height_ = 480;
        int rgb_fps_ = 30;
        cv::Mat rgb_intrinsic_;
        cv::Mat rgb_distortion_;

        /// Depth Parameter
        bool use_depth_ = true;
        int depth_width_ = 640;
        int depth_height_ = 480;
        int depth_fps_ = 30;
        cv::Mat depth_intrinsic_;
        cv::Mat depth_distortion_;

        /// IMU Parameter
        bool use_imu_ = true;
        int imu_fre_ = 100;

        /// External Parameter
        cv::Mat R_imu_rgb_;
        cv::Mat t_imu_rgb_;
        cv::Mat R_left_right;
        cv::Mat t_left_right;

        friend std::ostream &operator<<(std::ostream &os, const Options &options);
    };

    struct DeviceInfo {

        std::string name;                           // Friendly name
        std::string serial_number;                  // Device serial number
        std::string firmware_version;               // Primary firmware version
        std::string recommended_firmware_version;   // Recommended firmware version
        std::string physical_port;                  // Unique port identifier
        std::string debug_op_code;                  // Debug operation code
        bool advanced_mode;                         // True if in advanced mode
        std::string product_id;                     // Product ID
        bool camera_locked;                         // True if EEPROM is locked
        std::string usb_type_descriptor;            // USB specification (USB2/USB3)
        std::string product_line;                   // Device product line
        std::string asic_serial_number;             // ASIC serial number
        std::string firmware_update_id;             // Firmware update ID
        std::string ip_address;                     // IP address for remote camera

        bool support_rgb = false;
        bool support_depth = false;
        bool support_acc = false;
        bool support_gyro = false;

        // 重载 << 操作符
        friend std::ostream& operator<<(std::ostream& os, const DeviceInfo& deviceInfo);
    };

    using Ptr = std::shared_ptr<RealsenseD435i>;

    explicit RealsenseD435i(Options  options);

    ~RealsenseD435i();

    void initPipLine();

    // 获取彩色图
    bool getColorImage(cv::Mat& color_frame, double& timestamp);

    // 获取左右两个深度图
    bool getGrayImages(cv::Mat& left_gray_image, cv::Mat& right_gray_frame, double& timestamp);

    // 获取左侧灰度图
    bool getLeftDepthImage(cv::Mat& left_gray_image, double& timestamp);

    // 获取右侧灰度图
    bool getRightDepthImage(cv::Mat& right_gray_frame, double& timestamp);

    // 获取同步的彩色以及深度图
    bool getAlignedColorAndDepth(cv::Mat& color_frame, cv::Mat& depth_frame, double& timestamp);


    bool getIMU(double* acc, double* gyro, double& timestamp);

private:

    void getDeviceInfo() {
        rs2::context ctx;
        for (rs2::device dev: ctx.query_devices()) {

            if (std::string(dev.get_info(RS2_CAMERA_INFO_NAME)) == "Intel RealSense D435I") {


                // 填充 DeviceInfo 对象
                deviceInfo_.name = dev.get_info(RS2_CAMERA_INFO_NAME);
                deviceInfo_.serial_number = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
                deviceInfo_.firmware_version = dev.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION);
                deviceInfo_.recommended_firmware_version = dev.get_info(RS2_CAMERA_INFO_RECOMMENDED_FIRMWARE_VERSION);
                deviceInfo_.physical_port = dev.get_info(RS2_CAMERA_INFO_PHYSICAL_PORT);
                deviceInfo_.debug_op_code = dev.get_info(RS2_CAMERA_INFO_DEBUG_OP_CODE);
                deviceInfo_.advanced_mode = (dev.get_info(RS2_CAMERA_INFO_ADVANCED_MODE) == "1");
                deviceInfo_.product_id = dev.get_info(RS2_CAMERA_INFO_PRODUCT_ID);
                deviceInfo_.camera_locked = (dev.get_info(RS2_CAMERA_INFO_CAMERA_LOCKED) == "1");
                deviceInfo_.usb_type_descriptor = dev.get_info(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR);
                deviceInfo_.product_line = dev.get_info(RS2_CAMERA_INFO_PRODUCT_LINE);
                deviceInfo_.asic_serial_number = dev.get_info(RS2_CAMERA_INFO_ASIC_SERIAL_NUMBER);
                deviceInfo_.firmware_update_id = dev.get_info(RS2_CAMERA_INFO_FIRMWARE_UPDATE_ID);
                /*deviceInfo_.ip_address = std::string(dev.get_info(RS2_CAMERA_INFO_IP_ADDRESS));*/

                // 查询dev的支持的sensor的stream信息
                for (const auto &sensor: dev.query_sensors()) {

                    for (const auto &profile: sensor.get_stream_profiles()) {

                        if (profile.stream_type() == RS2_STREAM_GYRO)
                            deviceInfo_.support_gyro = true;

                        if (profile.stream_type() == RS2_STREAM_ACCEL)
                            deviceInfo_.support_acc = true;

                        if (profile.stream_type() == RS2_STREAM_COLOR)
                            deviceInfo_.support_rgb = true;

                        if (profile.stream_type() == RS2_STREAM_DEPTH)
                            deviceInfo_.support_depth = true;

                    }
                }

            }
        }

    }

    void setImagePipline();

    void setIMUPipline();

public:
    /// 用户传入参数
    Options options_;

    DeviceInfo deviceInfo_;

private:
    ///  图像管线
    rs2::colorizer color_map_;                      // rs
    rs2::config image_cfg_;                         // rs的图像配置文件
    rs2::pipeline image_pipe_;                      // rs的图像管线
    rs2::pipeline_profile rs_image_pipe_profile_;   // rs管道的配置文件

    rs2::video_stream_profile color_stream_;        // 彩色图数据流
    rs2::video_stream_profile depth_stream_;        // 深度图数据流
    rs2::hole_filling_filter hole_filler_;          // 深度图空洞填充滤波器
    rs2::align align_to_color_ = rs2::align(rs2_stream::RS2_STREAM_COLOR);     // 深度图向彩色图对其
    rs2::align align_to_depth_ = rs2::align(rs2_stream::RS2_STREAM_DEPTH);     // 彩色图向深度图对其

    //  IMU管线
    rs2::config imu_config_;
    rs2::pipeline imu_pipe_;                        // rs的IMU管线
    rs2::pipeline_profile rs_imu_pipe_profile_;     // rs管道的配置状态

    // sensor属性
    rs2_intrinsics rgb_property_{};
    rs2_intrinsics left_depth_property_{};
    rs2_intrinsics right_depth_property_{};
};





#endif //CAMERA_PRODUCER_CAMERA_REALSENSE_H
