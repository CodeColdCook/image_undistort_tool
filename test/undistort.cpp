#include <experimental/filesystem>

#include "camodocal/camera_models/CameraFactory.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "ros/ros.h"
#include "sensor_msgs/Image.h"

#define DEG_TO_RAD (M_PI / 180.0)

Eigen::Vector3d cameraRotation;
camodocal::CameraPtr cam;
std::string camFilePath;
cv::Mat undistMap;
int imgWidth = 1920;
int imgHeight = 1080;
double fov = 0;  // in degree
std::string image_source_path;
/**
 * @brief
 *
 * @param p_cam
 * @param rotation rotational offset from normal
 * @param imgWidth
 * @param imgHeight
 * @param f_center focal length in pin hole camera camera_mode (pixels are 1
 * unit sized)
 * @return CV_32FC2 mapping matrix
 */
cv::Mat GenOneUndistMap(camodocal::CameraPtr p_cam, Eigen::AngleAxisd rotation,
                        const unsigned &imgWidth, const unsigned &imgHeight,
                        const double &f_center) {
    cv::Mat map = cv::Mat(imgHeight, imgWidth, CV_32FC2);
    ROS_INFO("Generating map of size (%d,%d)", map.size[0], map.size[1]);
    ROS_INFO("Perspective facing (%.2f,%.2f,%.2f)",
             rotation._transformVector(Eigen::Vector3d(0, 0, 1))[0],
             rotation._transformVector(Eigen::Vector3d(0, 0, 1))[1],
             rotation._transformVector(Eigen::Vector3d(0, 0, 1))[2]);
    for (int x = 0; x < imgWidth; x++)
        for (int y = 0; y < imgHeight; y++) {
            Eigen::Vector3d objPoint =
                rotation * Eigen::Vector3d(((double)x - (double)imgWidth / 2),
                                           ((double)y - (double)imgHeight / 2),
                                           f_center);
            Eigen::Vector2d imgPoint;
            p_cam->spaceToPlane(objPoint, imgPoint);
            map.at<cv::Vec2f>(cv::Point(x, y)) =
                cv::Vec2f(imgPoint.x(), imgPoint.y());
        }

    ROS_INFO("Upper corners: (%.2f, %.2f), (%.2f, %.2f)",
             map.at<cv::Vec2f>(cv::Point(0, 0))[0],
             map.at<cv::Vec2f>(cv::Point(0, 0))[1],
             map.at<cv::Vec2f>(cv::Point(imgWidth - 1, 0))[0],
             map.at<cv::Vec2f>(cv::Point(imgWidth - 1, 0))[1]);
    return map;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "undistort");
    ros::NodeHandle nh;

    // obtain camera intrinsics
    nh.param<std::string>("cam_file", camFilePath,
                          "/home/wty/ws/src/fisheye-flattener/config/004.yaml");
    ROS_INFO("Camere file: %s", camFilePath.c_str());
    if (!std::experimental::filesystem::exists(camFilePath)) {
        ROS_ERROR("Camera file does not exist.");
        return 0;
    }
    cam = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(
        camFilePath);

    // remapping parameters
    cameraRotation.x() = 0;
    cameraRotation.y() = 0;
    cameraRotation.z() = 0;
    nh.param<double>("fov", fov, 90);
    nh.param<int>("imgWidth", imgWidth, 1920);
    nh.param<int>("imgHeight", imgHeight, 1080);
    if (imgWidth <= 0) {
        ROS_ERROR("Resolution must be non-negative");
        return 0;
    }
    if (fov < 0) {
        ROS_ERROR("FOV must be non-negative");
        return 0;
    }

    nh.param<std::string>("image_source_path", image_source_path,
                          "/home/wty/ws/selected_004/left03.jpg");
    double center_roi_fov = 130;
    double f_center =
        (double)imgWidth / 2.0 / tan(center_roi_fov * DEG_TO_RAD / 2);
    Eigen::AngleAxisd roi_direction;
    roi_direction = Eigen::AngleAxis<double>(cameraRotation.norm(), cameraRotation.normalized())
            .inverse();

    undistMap = GenOneUndistMap(cam, roi_direction, imgWidth, imgWidth, f_center);
    cv::Mat image = cv::imread(image_source_path);
    if (image.empty()) {
        std::cout << "cv::imread " << image_source_path.c_str() << " failed.\n";
        return 0;
    }

    cv::Mat out_image;
    cv::remap(image, out_image, undistMap, cv::Mat(), cv::INTER_LINEAR);
    cv::imwrite("/home/wty/ws/selected_004/fisheye_undistort.jpg", out_image);





    return 0;
}
