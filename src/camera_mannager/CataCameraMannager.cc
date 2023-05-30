
#include "camodocal/camera_mannager/CataCameraMannager.h"

namespace camodocal {

bool CataCameraMannager::Init() {
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
}
}  // namespace camodocal