#include <ros/ros.h>

#include <boost/filesystem.hpp>
#include <csignal>
#include <vector>

#include "camera_mannager.h"

FisheyeUndistort::Ptr fisheye_undistorter_;

void signalHandler(int sig) {
  ROS_WARN("Trying to exit!");
  ros::shutdown();
}

bool try_load_image(const std::string &file_name, cv::Mat &img) {
  try {
    img = cv::imread(file_name);
  } catch (const cv::Exception &e) {
    std::cerr << e.what() << std::endl;
    return false;
  }

  if (img.empty()) {
    std::cout << "cv::imread " << file_name.c_str() << " failed.\n";
    return false;
  }
  return true;
}

bool check_for_save(const std::string &path, cv::Mat *image_src,
                    cv::Mat *image_dst, bool print_tips) {
  cv::imshow("image src", *image_src);
  cv::imshow("image undistorted", *image_dst);

  if (print_tips)
    std::cout << "*** Press "
              << "\n [s] or [S] for save "
              << "\n [p] or [P] for pass "
              << "\n [q] or [Q] or [Esc] for quit " << std::endl;
  int key = cv::waitKey(0);
  if (key == 's' || key == 'S') {
    cv::imwrite(path, *image_dst);
    std::cout << "*** undistorted image has saved, path: " << std::endl
              << path << std::endl;
  } else if (key == 'p' || key == 'P') {
    std::cout << "*** undistorted image is not saved, next frame" << std::endl;
  } else {
    std::cout << "*** undistorted image is not saved, quit now" << std::endl;
    return false;
  }

  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "undistort_image");
  ros::NodeHandle nh;

  FisheyeUndistortconfig config;
  nh.getParam("/fisheye/imgWidth", config.imgWidth);
  nh.getParam("/fisheye/imgHeight", config.imgHeight);
  nh.getParam("/fisheye/fov_camera", config.fov_camera);
  nh.getParam("/fisheye/fov_center_roi", config.fov_center_roi);
  nh.getParam("/fisheye/cam_file", config.camFilePath);
  nh.getParam("/fisheye/undistort_scale", config.scale);
  config.cameraRotation.x() = 0;
  config.cameraRotation.y() = 0;
  config.cameraRotation.z() = 0;

  std::cout << "imgWidth: " << config.imgWidth << std::endl;
  std::cout << "imgHeight: " << config.imgHeight << std::endl;
  std::cout << "fov_camera: " << config.fov_camera << std::endl;
  std::cout << "fov_center_roi: " << config.fov_center_roi << std::endl;
  std::cout << "cam_file: " << config.camFilePath << std::endl;
  std::cout << "undistort_scale: " << config.scale << std::endl;

  fisheye_undistorter_ = FisheyeUndistort::Ptr(new FisheyeUndistort(config));
  camodocal::CataCamera::Parameters params_raw =
      fisheye_undistorter_->p_cam_->getParameters();
  cv::Mat remap_mat_1, remap_mat_2;
  cv::Mat K_new = fisheye_undistorter_->p_cam_->initUndistortRectifyMap(
      remap_mat_1, remap_mat_2, params_raw.gamma1() / config.scale,
      params_raw.gamma2() / config.scale, cv::Size(0, 0), params_raw.u0(),
      params_raw.v0());

  camodocal::CataCamera::Parameters param_new;
  param_new.gamma1() = K_new.at<float>(0, 0);
  param_new.gamma2() = K_new.at<float>(1, 1);
  param_new.u0() = K_new.at<float>(0, 2);
  param_new.v0() = K_new.at<float>(1, 2);
  param_new.imageWidth() = 1920;
  param_new.imageHeight() = 1080;
  fisheye_undistorter_->p_cam_->setParameters(param_new);

  std::cout << "param_raw: " << params_raw << std::endl;
  std::cout << "param_new: " << param_new << std::endl;

  std::string path_image_src, path_image_dst;
  nh.getParam("undistort_image/path_image_src", path_image_src);
  nh.getParam("undistort_image/path_image_dst", path_image_dst);
  std::cout << "path_image_src: " << path_image_src << std::endl;
  std::cout << "path_image_dst: " << path_image_dst << std::endl;

  cv::namedWindow("image src", 0);
  cv::resizeWindow("image src", 640, 480);
  cv::namedWindow("image undistorted", 0);
  cv::resizeWindow("image undistorted", 640, 480);

  int multi_image, check_first_frame, check_every_frame;
  nh.getParam("/undistort_image/multi_image", multi_image);
  nh.getParam("/undistort_image/check_first_frame", check_first_frame);
  nh.getParam("/undistort_image/check_every_frame", check_every_frame);

  if (multi_image > 0) {
    std::string folder_image_src, folder_image_dst;
    nh.getParam("/undistort_image/folder_image_src", folder_image_src);
    nh.getParam("/undistort_image/folder_image_dst", folder_image_dst);

    // look for images in input directory
    std::vector<std::string> imageFilenames;
    boost::filesystem::directory_iterator itr;
    for (boost::filesystem::directory_iterator itr(folder_image_src);
         itr != boost::filesystem::directory_iterator(); ++itr) {
      if (!boost::filesystem::is_regular_file(itr->status())) {
        continue;
      }
      std::string filename = itr->path().filename().string();
      imageFilenames.push_back(itr->path().string());
    }
    if (imageFilenames.empty()) {
      std::cerr << "# ERROR: No chessboard images found." << std::endl;
      return 1;
    }
    std::sort(imageFilenames.begin(), imageFilenames.end());

    for (size_t i = 0; i < imageFilenames.size(); ++i) {
      cv::Mat image_src;
      if (!try_load_image(imageFilenames.at(i), image_src)) continue;

      if (image_src.cols != config.imgWidth ||
          image_src.rows != config.imgHeight) {
        cv::resize(image_src, image_src,
                   cv::Size(config.imgWidth, config.imgHeight));
      }

      cv::Mat image_dst;
      cv::remap(image_src, image_dst, remap_mat_1, remap_mat_2,
                cv::INTER_LINEAR);

      std::string save_path_cur =
          folder_image_dst + "/" + "undistorted_" + std::to_string(i) + ".jpg";
      if (i == 0 && check_first_frame > 0) {
        if (!check_for_save(save_path_cur, &image_src, &image_dst, true))
          return 0;
      } else if (check_every_frame > 0) {
        if (i == 0)
          if (!check_for_save(save_path_cur, &image_src, &image_dst, true))
            return 0;
        if (!check_for_save(save_path_cur, &image_src, &image_dst, false))
          return 0;
      } else {
        cv::imshow("image src", image_src);
        cv::imshow("image undistorted", image_dst);
        cv::waitKey(30);
        cv::imwrite(save_path_cur, image_dst);
      }
    }
  }

  cv::destroyAllWindows();
  signal(SIGINT, signalHandler);
  return 0;
}