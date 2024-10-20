#include "realsense_pipline.h"

#include <utility>


std::ostream &operator<<(std::ostream &os, const RealsenseD435i::DeviceInfo &deviceInfo) {
    os << "Device Info:\n"
       << "  Name: " << deviceInfo.name << "\n"
       << "  Serial Number: " << deviceInfo.serial_number << "\n"
       << "  Firmware Version: " << deviceInfo.firmware_version << "\n"
       << "  Recommended Firmware Version: " << deviceInfo.recommended_firmware_version << "\n"
       << "  Physical Port: " << deviceInfo.physical_port << "\n"
       << "  Debug OP Code: " << deviceInfo.debug_op_code << "\n"
       << "  Advanced Mode: " << (deviceInfo.advanced_mode ? "true" : "false") << "\n"
       << "  Product ID: " << deviceInfo.product_id << "\n"
       << "  Camera Locked: " << (deviceInfo.camera_locked ? "true" : "false") << "\n"
       << "  USB Type Descriptor: " << deviceInfo.usb_type_descriptor << "\n"
       << "  Product Line: " << deviceInfo.product_line << "\n"
       << "  ASIC Serial Number: " << deviceInfo.asic_serial_number << "\n"
       << "  Firmware Update ID: " << deviceInfo.firmware_update_id << "\n"
       << "  IP Address: " << deviceInfo.ip_address << "\n"
       << "  Support RGB: " << (deviceInfo.support_rgb ? "true" : "false") << "\n"
       << "  Support Depth: " << (deviceInfo.support_depth ? "true" : "false") << "\n"
       << "  Support Accelerometer: " << (deviceInfo.support_acc ? "true" : "false") << "\n"
       << "  Support Gyroscope: " << (deviceInfo.support_gyro ? "true" : "false") << "\n";
    return os;
}

std::ostream &operator<<(std::ostream &os, const RealsenseD435i::Options &options) {
    os << "Options:\n"
       << "  Use RGB: " << (options.use_rgb_ ? "true" : "false") << "\n"
       << "  RGB Width: " << options.rgb_width_ << "\n"
       << "  RGB Height: " << options.rgb_height_ << "\n"
       << "  RGB FPS: " << options.rgb_fps_ << "\n"
       << "  Use Depth: " << (options.use_depth_ ? "true" : "false") << "\n"
       << "  Depth Width: " << options.depth_width_ << "\n"
       << "  Depth Height: " << options.depth_height_ << "\n"
       << "  Depth FPS: " << options.depth_fps_ << "\n"
       << "  Use IMU: " << (options.use_imu_ ? "true" : "false") << "\n"
       << "  IMU Frequency: " << options.imu_fre_ << "\n";

    // 打印相机内参和畸变参数
    if (!options.rgb_intrinsic_.empty()) {
        os << "  RGB Intrinsic:\n" << options.rgb_intrinsic_ << "\n";
    } else {
        os << "  RGB Intrinsic: Not set\n";
    }

    if (!options.rgb_distortion_.empty()) {
        os << "  RGB Distortion:\n" << options.rgb_distortion_ << "\n";
    } else {
        os << "  RGB Distortion: Not set\n";
    }

    if (!options.depth_intrinsic_.empty()) {
        os << "  Depth Intrinsic:\n" << options.depth_intrinsic_ << "\n";
    } else {
        os << "  Depth Intrinsic: Not set\n";
    }

    if (!options.depth_distortion_.empty()) {
        os << "  Depth Distortion:\n" << options.depth_distortion_ << "\n";
    } else {
        os << "  Depth Distortion: Not set\n";
    }

    // 打印外部参数
    if (!options.R_imu_rgb_.empty()) {
        os << "  R IMU to RGB:\n" << options.R_imu_rgb_ << "\n";
    } else {
        os << "  R IMU to RGB: Not set\n";
    }

    if (!options.t_imu_rgb_.empty()) {
        os << "  t IMU to RGB:\n" << options.t_imu_rgb_ << "\n";
    } else {
        os << "  t IMU to RGB: Not set\n";
    }

    if (!options.R_left_right.empty()) {
        os << "  R Left to Right:\n" << options.R_left_right << "\n";
    } else {
        os << "  R Left to Right: Not set\n";
    }

    if (!options.t_left_right.empty()) {
        os << "  t Left to Right:\n" << options.t_left_right << "\n";
    } else {
        os << "  t Left to Right: Not set\n";
    }

    return os;
}


RealsenseD435i::RealsenseD435i(Options  options) : options_(std::move(options)){

    std::cout << options << std::endl;

    getDeviceInfo();
    std::cout << deviceInfo_ << std::endl;

    initPipLine();
}

RealsenseD435i::~RealsenseD435i()  {
    image_pipe_.stop();
    imu_pipe_.stop();
}

void RealsenseD435i::initPipLine(){

    /// 配置image管道
    setImagePipline();

    /// 配置imu管道
    setIMUPipline();

}

void RealsenseD435i::setImagePipline() {

    if(!options_.use_rgb_ && !options_.use_depth_){
        return;
    }

    image_cfg_.enable_device(deviceInfo_.serial_number); // 激活的设备序列号

    //  配置彩色图片
    if (deviceInfo_.support_rgb && options_.use_rgb_) {
        image_cfg_.enable_stream(RS2_STREAM_COLOR, options_.rgb_width_, options_.rgb_height_, RS2_FORMAT_RGB8, options_.rgb_fps_);
    }

    //  配置深度图片
    if(deviceInfo_.support_depth && options_.use_depth_){
        image_cfg_.enable_stream(RS2_STREAM_INFRARED, 1, options_.depth_width_, options_.depth_height_, RS2_FORMAT_Y8, options_.depth_fps_);
        image_cfg_.enable_stream(RS2_STREAM_INFRARED, 2, options_.depth_width_, options_.depth_height_, RS2_FORMAT_Y8, options_.depth_fps_);
        image_cfg_.enable_stream(RS2_STREAM_DEPTH, options_.depth_width_, options_.depth_height_, RS2_FORMAT_Z16, options_.depth_fps_);
    }

    //  开启数据stream
    rs_image_pipe_profile_ = image_pipe_.start(image_cfg_);

/*    // 配置彩色相机曝光
    rs2::sensor sensor = rs_image_pipe_profile_.get_device().first<rs2::color_sensor>();
    sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1);
    sensor.set_option(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE, 1);
    */

}

void RealsenseD435i::setIMUPipline() {

    if(!options_.use_imu_){
        return;
    }

    if(options_.use_imu_ && deviceInfo_.support_acc && deviceInfo_.support_gyro){
        // 配置IMU的流参数
        imu_config_.enable_device(deviceInfo_.serial_number); // 激活的设备序列号
        imu_config_.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
        imu_config_.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
        rs_imu_pipe_profile_ = imu_pipe_.start(imu_config_);
    }

}

bool RealsenseD435i::getAlignedColorAndDepth(cv::Mat& color_frame, cv::Mat &depth_frame, double &timestamp) {

    // 等待获取帧集合
    rs2::frameset frames = image_pipe_.wait_for_frames();

    // 获取时间戳
    timestamp = frames.get_timestamp() /*单位为ms*/ * 1e-3;

    // 彩色图和深度图对齐
    auto aligned_frames = align_to_color_.process(frames);
    rs2::video_frame color_img = aligned_frames.get_color_frame();
    rs2::depth_frame depth_map = aligned_frames.get_depth_frame();

    // 转换为OpenCV格式
    auto color_tmp = cv::Mat(cv::Size(options_.rgb_width_, options_.rgb_height_), CV_8UC3, (void*)color_img.get_data(), cv::Mat::AUTO_STEP);
    cv::cvtColor(color_tmp, color_frame, cv::COLOR_RGB2BGR);

    depth_map = hole_filler_.process(depth_map);
    depth_frame = cv::Mat(cv::Size(options_.depth_width_, options_.depth_height_), CV_16UC1, (void*)depth_map.get_data(), cv::Mat::AUTO_STEP);

    return true;
}

bool RealsenseD435i::getColorImage(cv::Mat &color_frame, double &timestamp) {
    // 等待获取帧集合
    rs2::frameset frames = image_pipe_.wait_for_frames();

    // 获取时间戳
    timestamp = frames.get_timestamp() /*单位为ms*/ * 1e-3;
    rs2::video_frame color_img = frames.get_color_frame();

    // 转换为OpenCV格式
    auto color_tmp = cv::Mat(cv::Size(options_.rgb_width_, options_.rgb_height_), CV_8UC3, (void*)color_img.get_data(), cv::Mat::AUTO_STEP);
    cv::cvtColor(color_tmp, color_frame, cv::COLOR_RGB2BGR);

    return true;
}

bool RealsenseD435i::getGrayImages(cv::Mat &left_gray_image, cv::Mat &right_gray_frame, double &timestamp) {

    // 等待获取帧集合
    rs2::frameset frames = image_pipe_.wait_for_frames();

    // 获取时间戳
    timestamp = frames.get_timestamp() /*单位为ms*/ * 1e-3;

    auto ir_frame_left = frames.get_infrared_frame(1);
    auto ir_frame_right = frames.get_infrared_frame(2);

    left_gray_image = cv::Mat(cv::Size(options_.depth_width_, options_.depth_height_), CV_8UC1, (void *) ir_frame_left.get_data(), cv::Mat::AUTO_STEP);
    right_gray_frame = cv::Mat(cv::Size(options_.depth_width_, options_.depth_height_), CV_8UC1, (void *) ir_frame_right.get_data(), cv::Mat::AUTO_STEP);

    return true;
}


bool RealsenseD435i::getIMU(double* acc, double* gyro, double& timestamp) {
    // 等待获取帧集合
    rs2::frameset frames = imu_pipe_.wait_for_frames();

    // 初始化标志，用于检查是否获取到加速度计和陀螺仪数据
    bool found_gyro = false;
    bool found_accel = false;

    // 遍历帧集合中的每个帧
    for (const rs2::frame& frame : frames) {
        if (frame.is<rs2::motion_frame>()) {
            rs2::motion_frame motion = frame.as<rs2::motion_frame>();
            timestamp = motion.get_timestamp() * 1e-3;  // 将时间戳转换为秒

            if (motion.get_profile().stream_type() == RS2_STREAM_GYRO && motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F) {
                rs2_vector gyro_data = motion.get_motion_data();
                gyro[0] = gyro_data.x;
                gyro[1] = gyro_data.y;
                gyro[2] = gyro_data.z;
                found_gyro = true;
            }

            if (motion.get_profile().stream_type() == RS2_STREAM_ACCEL && motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F) {
                rs2_vector accel_data = motion.get_motion_data();
                acc[0] = accel_data.x;
                acc[1] = accel_data.y;
                acc[2] = accel_data.z;
                found_accel = true;
            }
        }
    }

    // 返回 true 表示成功获取到加速度计和陀螺仪数据
    return found_gyro && found_accel;
}


