#ifndef CAMERAMANNAGER_H
#define CAMERAMANNAGER_H

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "camodocal/camera_models/Camera.h"

namespace camodocal {
class CameraMannager {
 public:
  std::string config_path_;
  float scale_;

 protected:
  int imageWidth_;
  int imageHeight_;
  cv::Mat remap_mat_1_, remap_mat_2_;
  cv::Mat K_scaled_;

  virtual bool Init() = 0;

  CameraMannager(const std::string &config_path, const float &scale)
      : config_path_(config_path), scale_(scale) {}

 public:
  bool UndistortImage(const cv::Mat &image_src, cv::Mat &image_dst,
                      bool auto_scale = true) const;
};

}  // namespace camodocal
#endif