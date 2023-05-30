
#include "camodocal/camera_mannager/PinholeCameraMannager.h"

namespace camodocal {

bool PinholeCameraMannager::Init() {
  camera_raw_ptr_ = camodocal::PinholeCameraPtr(new camodocal::PinholeCamera);
  camera_scaled_ptr_ = camodocal::PinholeCameraPtr(new camodocal::PinholeCamera);

  params_raw_.readFromYamlFile(config_path_);
  camera_raw_ptr_->setParameters(params_raw_);
  imageWidth_ = camera_raw_ptr_->imageWidth();
  imageHeight_ = camera_raw_ptr_->imageHeight();

  K_scaled_ = camera_raw_ptr_->initUndistortRectifyMap(
      remap_mat_1_, remap_mat_2_, params_raw_.fx() / scale_,
      params_raw_.fy() / scale_, cv::Size(0, 0), params_raw_.cx(),
      params_raw_.cy());

  params_scaled_.fx() = K_scaled_.at<float>(0, 0);
  params_scaled_.fy() = K_scaled_.at<float>(1, 1);
  params_scaled_.cx() = K_scaled_.at<float>(0, 2);
  params_scaled_.cy() = K_scaled_.at<float>(1, 2);
  params_scaled_.imageWidth() = imageWidth_;
  params_scaled_.imageHeight() = imageHeight_;
  camera_scaled_ptr_->setParameters(params_scaled_);

  std::cout << "param raw: " << params_raw_ << std::endl;
  std::cout << "param scaled: " << params_scaled_ << std::endl;
}
}  // namespace camodocal