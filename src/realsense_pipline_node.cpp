
#include "realsense_pipline.h"
#include "yaml-cpp/yaml.h"
#include "thread"



int main(int argc, char *argv[]) {

    // 解析Yaml参数
    std::string yaml_path = std::string(ROOT_DIR) + "config/realsense_d435i.yaml";
    RealsenseD435i::Options options;
    try {
        YAML::Node config = YAML::LoadFile(yaml_path);

        // 解析 RGB 参数
        options.use_rgb_ = config["rgb"]["open"].as<bool>();
        options.rgb_width_ = config["rgb"]["width"].as<int>();
        options.rgb_height_ = config["rgb"]["height"].as<int>();
        options.rgb_fps_ = config["rgb"]["fps"].as<int>();

        // 解析 RGB 内部参数
        auto rgb_intrinsic = config["rgb"]["intrinsic"];
        options.rgb_intrinsic_ = cv::Mat(3, 3, CV_64F, rgb_intrinsic.as<std::vector<double>>().data());

        auto rgb_distortion = config["rgb"]["distortion"];
        options.rgb_distortion_ = cv::Mat(1, 5, CV_64F, rgb_distortion.as<std::vector<double>>().data());

        // 解析 Depth 参数
        options.use_depth_ = config["depth"]["open"].as<bool>();
        options.depth_width_ = config["depth"]["width"].as<int>();
        options.depth_height_ = config["depth"]["height"].as<int>();
        options.depth_fps_ = config["depth"]["fps"].as<int>();

        // 解析 Depth 内部参数
        auto depth_intrinsic = config["depth"]["intrinsic"];
        options.depth_intrinsic_ = cv::Mat(3, 3, CV_64F, depth_intrinsic.as<std::vector<double>>().data());

        auto depth_distortion = config["depth"]["distortion"];
        options.depth_distortion_ = cv::Mat(1, 5, CV_64F, depth_distortion.as<std::vector<double>>().data());

        // 解析 IMU 参数
        options.use_imu_ = config["imu"]["open"].as<bool>();
        options.imu_fre_ = config["imu"]["fps"].as<int>();

        // 解析外部参数
        auto T_left_right = config["external"]["T_left_right"];
        options.R_left_right = cv::Mat(4, 4, CV_64F, T_left_right.as<std::vector<double>>().data());

        // 如果需要解析 R_imu_rgb_ 和 t_imu_rgb_，请根据配置文件补充相应字段

    } catch (const std::exception &e) {
        std::cerr << "Error loading YAML file: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
    RealsenseD435i realsenseD435I(options);


    while (1) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));

/*
        cv::Mat color_image;
        cv::Mat depth_frame;
        double timestamp;

        realsenseD435I.getAlignedColorAndDepth(color_image, depth_frame, timestamp);


        // 进行显示
        cv::imshow("rgb", color_image);
        cv::imshow("depth", depth_frame);
        cv::waitKey(1);*/


/*        cv::Mat left_gray_image;
        cv::Mat right_gray_image;
        double timestamp;
        realsenseD435I.getGrayImages(left_gray_image, right_gray_image, timestamp);

        // 进行显示
        cv::imshow("left_gray", left_gray_image);
        cv::imshow("right_gray", right_gray_image);

        cv::waitKey(1);*/

        double acc[3], gyro[3], timestamp;
        realsenseD435I.getIMU(acc, gyro, timestamp);
        std::cout << acc[0] << " " << acc[1] << " " << acc[2] << std::endl;
        std::cout << gyro[0] << " " << gyro[1] << " " << gyro[2] << std::endl;
        std::cout << timestamp << std::endl;


    }


}