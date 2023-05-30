#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/code_utils/image_utils.h"
camodocal::CataCameraPtr p_cam_;

int main(int argc, char **argv) {
  std::string camera_config_path = "../data/cata_camera/intrinsic_cata.yaml";
  float scale = 5.2;
  std::string path_image_src, path_image_dst;
  path_image_src = "../data/cata_camera/cata_test.jpg";
  path_image_dst = "../data/cata_camera/cata_undistorted.jpg";

  p_cam_ = camodocal::CataCameraPtr(new camodocal::CataCamera);
  camodocal::CataCamera::Parameters params_raw = p_cam_->getParameters();
  params_raw.readFromYamlFile(camera_config_path);
  p_cam_->setParameters(params_raw);

  cv::Mat remap_mat_1, remap_mat_2;
  cv::Mat K_new = p_cam_->initUndistortRectifyMap(
      remap_mat_1, remap_mat_2, params_raw.gamma1() / scale,
      params_raw.gamma2() / scale, cv::Size(0, 0), params_raw.u0(),
      params_raw.v0());

  camodocal::CataCamera::Parameters param_new;
  param_new.gamma1() = K_new.at<float>(0, 0);
  param_new.gamma2() = K_new.at<float>(1, 1);
  param_new.u0() = K_new.at<float>(0, 2);
  param_new.v0() = K_new.at<float>(1, 2);
  param_new.imageWidth() = 1920;
  param_new.imageHeight() = 1080;
  p_cam_->setParameters(param_new);

  std::cout << "param_raw: " << params_raw << std::endl;
  std::cout << "param_new: " << param_new << std::endl;

  std::cout << "path_image_src: " << path_image_src << std::endl;
  std::cout << "path_image_dst: " << path_image_dst << std::endl;

  cv::Mat image_src;
  if (!try_load_image(path_image_src, image_src)) return 0;

  if (image_src.cols != param_new.imageWidth() ||
      image_src.rows != param_new.imageHeight()) {
    cv::resize(image_src, image_src,
               cv::Size(param_new.imageWidth(), param_new.imageHeight()));
  }

  cv::Mat image_dst;
  cv::remap(image_src, image_dst, remap_mat_1, remap_mat_2, cv::INTER_LINEAR);
  cv::imwrite(path_image_dst, image_dst);
  return 0;
}