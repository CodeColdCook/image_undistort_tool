#include "camodocal/camera_mannager/CameraMannager.h"

namespace camodocal {

bool CameraMannager::UndistortImage(const cv::Mat &image_src,
                                    cv::Mat &image_dst, bool auto_scale) const {
  if (image_src.cols != imageWidth_ || image_src.rows != imageHeight_) {
    if (!auto_scale) return false;
    cv::resize(image_src, image_dst, cv::Size(imageWidth_, imageHeight_));
    cv::remap(image_dst, image_dst, remap_mat_1_, remap_mat_2_,
              cv::INTER_LINEAR);
    return true;
  }
  cv::remap(image_src, image_dst, remap_mat_1_, remap_mat_2_, cv::INTER_LINEAR);
  return true;
}

}  // namespace camodocal