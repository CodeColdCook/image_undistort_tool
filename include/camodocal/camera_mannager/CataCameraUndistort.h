#ifndef CATACAMERAMANNAGER_H
#define CATACAMERAMANNAGER_H

#include "CameraMannager.h"
#include "camodocal/camera_models/CataCamera.h"

namespace camodocal {

class CataCameraUndistort {
 public:
 public:
  std::string config_path_;
  float scale_;
  int imageWidth_;
  int imageHeight_;
  cv::Mat remap_mat_1_, remap_mat_2_;
  cv::Mat K_scaled_;

  camodocal::CataCameraPtr camera_raw_ptr_;
  camodocal::CataCameraPtr camera_scaled_ptr_;

  camodocal::CataCamera::Parameters params_raw_;
  camodocal::CataCamera::Parameters params_scaled_;

  CataCameraUndistort(const std::string &config_path, const float &scale)
      : config_path_(config_path), scale_(scale) {
    camera_raw_ptr_ = camodocal::CataCameraPtr(new camodocal::CataCamera);
    camera_scaled_ptr_ = camodocal::CataCameraPtr(new camodocal::CataCamera);

    params_raw_.readFromYamlFile(config_path_);
    camera_raw_ptr_->setParameters(params_raw_);
    imageWidth_ = camera_raw_ptr_->imageWidth();
    imageHeight_ = camera_raw_ptr_->imageHeight();

    K_scaled_ = camera_raw_ptr_->initUndistortRectifyMap(
        remap_mat_1_, remap_mat_2_, params_raw_.gamma1() / scale_,
        params_raw_.gamma2() / scale_, cv::Size(0, 0), params_raw_.u0(),
        params_raw_.v0());

    params_scaled_.gamma1() = K_scaled_.at<float>(0, 0);
    params_scaled_.gamma2() = K_scaled_.at<float>(1, 1);
    params_scaled_.u0() = K_scaled_.at<float>(0, 2);
    params_scaled_.v0() = K_scaled_.at<float>(1, 2);
    params_scaled_.imageWidth() = imageWidth_;
    params_scaled_.imageHeight() = imageHeight_;
    camera_scaled_ptr_->setParameters(params_scaled_);

    std::cout << "param raw: " << params_raw_ << std::endl;
    std::cout << "param scaled: " << params_scaled_ << std::endl;
  };
  bool UndistortImage(const cv::Mat &image_src, cv::Mat &image_dst,
                      bool auto_scale = true) const {
    if (image_src.cols != imageWidth_ || image_src.rows != imageHeight_) {
      if (!auto_scale) return false;
      cv::resize(image_src, image_dst, cv::Size(imageWidth_, imageHeight_));
      cv::remap(image_dst, image_dst, remap_mat_1_, remap_mat_2_,
                cv::INTER_LINEAR);
      return true;
    }
    cv::remap(image_src, image_dst, remap_mat_1_, remap_mat_2_,
              cv::INTER_LINEAR);
    return true;
  }
};

typedef boost::shared_ptr<CataCameraUndistort> CataCameraUndistortPtr;
typedef boost::shared_ptr<const CataCameraUndistort>
    CataCameraUndistortConstPtr;
}  // namespace camodocal
#endif