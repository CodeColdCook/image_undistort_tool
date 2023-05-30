#include <gtest/gtest.h>

// #include "camodocal/camera_mannager/CataCameraMannager.h"
// #include "camodocal/camera_mannager/PinholeCameraMannager.h"
#include "camodocal/camera_mannager/CataCameraUndistort.h"
#include "camodocal/code_utils/image_utils.h"

TEST(CataCameraUndistort, UndistortImageCata) {
  std::string camera_config_path = "../data/cata_camera/intrinsic_cata.yaml";
  float scale = 5.2;
  std::string path_image_src, path_image_dst;
  path_image_src = "../data/cata_camera/cata_test.jpg";
  path_image_dst = "../data/cata_camera/cata_undistorted.jpg";
  std::cout << "path_image_src: " << path_image_src << std::endl;
  std::cout << "path_image_dst: " << path_image_dst << std::endl;

  camodocal::CataCameraUndistort camera_mannager_(camera_config_path, scale);

  cv::Mat image_src;
  EXPECT_TRUE(try_load_image(path_image_src, image_src));
  cv::Mat image_dst;
  EXPECT_TRUE(camera_mannager_.UndistortImage(image_src, image_dst));
  // std::cout << "UndistortImage failed" << std::endl;
  // return 0;
  cv::imwrite(path_image_dst, image_dst);
}

// TEST(CataCameraMannager, UndistortImageCata) {
//   std::string camera_config_path = "../data/cata_camera/intrinsic_cata.yaml";
//   float scale = 5.2;
//   std::string path_image_src, path_image_dst;
//   path_image_src = "../data/cata_camera/cata_test.jpg";
//   path_image_dst = "../data/cata_camera/cata_undistorted.jpg";
//   std::cout << "path_image_src: " << path_image_src << std::endl;
//   std::cout << "path_image_dst: " << path_image_dst << std::endl;

//   camodocal::CataCameraMannager camera_mannager_(camera_config_path, scale);

//   cv::Mat image_src;
//   EXPECT_TRUE(try_load_image(path_image_src, image_src));
//   cv::Mat image_dst;
//   EXPECT_TRUE(camera_mannager_.UndistortImage(image_src, image_dst));
//   // std::cout << "UndistortImage failed" << std::endl;
//   // return 0;
//   cv::imwrite(path_image_dst, image_dst);
// }

// TEST(PinholeCameraMannager, UndistortImagePinhole) {
//   std::string camera_config_path =
//       "../data/pinhole_camera/intrinsic_pinhole.yaml";
//   float scale = 1.0;
//   std::string path_image_src, path_image_dst;
//   path_image_src = "../data/pinhole_camera/pinhole_test.jpg";
//   path_image_dst = "../data/pinhole_camera/pinhole_undistorted.jpg";
//   std::cout << "path_image_src: " << path_image_src << std::endl;
//   std::cout << "path_image_dst: " << path_image_dst << std::endl;

//   camodocal::PinholeCameraMannager camera_mannager_(camera_config_path, scale);

//   cv::Mat image_src;
//   EXPECT_TRUE(try_load_image(path_image_src, image_src));
//   cv::Mat image_dst;
//   EXPECT_TRUE(camera_mannager_.UndistortImage(image_src, image_dst));
//   // std::cout << "UndistortImage failed" << std::endl;
//   // return 0;
//   cv::imwrite(path_image_dst, image_dst);
// }

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}