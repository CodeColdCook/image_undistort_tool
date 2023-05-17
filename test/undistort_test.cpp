#include <ros/ros.h>

#include <vector>

#include "camera_mannager.h"

int ROW = 1080;
int COL = 1920;
int FOCAL_LENGTH = 920;

FisheyeUndistort::Ptr fisheye_undistorter_;
cv::Mat img_show_undis = cv::Mat(cv::Size(1920, 1080), CV_8UC3);

void showUndistortion() {
  cv::Mat undistortedImg(ROW + 600, COL + 600, CV_8UC1, cv::Scalar(0));
  std::vector<Eigen::Vector2d> distortedp, undistortedp;
  for (int i = 0; i < COL; i++)
    for (int j = 0; j < ROW; j++) {
      Eigen::Vector2d a(i, j);
      Eigen::Vector3d b;
      fisheye_undistorter_->p_cam_->liftProjective(a, b);
      distortedp.push_back(a);
      undistortedp.push_back(Eigen::Vector2d(b.x() / b.z(), b.y() / b.z()));
      // printf("%f,%f->%f,%f,%f\n)\n", a.x(), a.y(), b.x(), b.y(),
      // b.z());
    }
  for (int i = 0; i < int(undistortedp.size()); i++) {
    cv::Mat pp(3, 1, CV_32FC1);
    pp.at<float>(0, 0) = undistortedp[i].x() * FOCAL_LENGTH + COL / 2;
    pp.at<float>(1, 0) = undistortedp[i].y() * FOCAL_LENGTH + ROW / 2;
    pp.at<float>(2, 0) = 1.0;
    // cout << trackerData[0].K << endl;
    // printf("%lf %lf\n", p.at<float>(1, 0), p.at<float>(0, 0));
    // printf("%lf %lf\n", pp.at<float>(1, 0), pp.at<float>(0, 0));
    if (pp.at<float>(1, 0) + 300 >= 0 && pp.at<float>(1, 0) + 300 < ROW + 600 &&
        pp.at<float>(0, 0) + 300 >= 0 && pp.at<float>(0, 0) + 300 < COL + 600) {
      undistortedImg.at<uchar>(pp.at<float>(1, 0) + 300,
                               pp.at<float>(0, 0) + 300) =
          img_show_undis.at<uchar>(distortedp[i].y(), distortedp[i].x());
    } else {
      // ROS_ERROR("(%f %f) -> (%f %f)", distortedp[i].y, distortedp[i].x,
      // pp.at<float>(1, 0), pp.at<float>(0, 0));
    }
  }
  cv::imshow("undistored_lift", undistortedImg);
  cv::imwrite(
      "/home/wty/calib_ws_3/src/fisheye-flattener/data/selected_004/"
      "undistored_lift.jpg",
      undistortedImg);
  cv::waitKey(0);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "undistort");
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
  fisheye_undistorter_->MakeUndistMap2();

  std::string image_src_path, image_dst_path;
  nh.getParam("test/image_src_path", image_src_path);
  nh.getParam("test/image_dst_path", image_dst_path);
  std::cout << "image_src_path: " << image_src_path << std::endl;
  std::cout << "image_dst_path: " << image_dst_path << std::endl;

  cv::Mat image = cv::imread(image_src_path);
  if (image.empty()) {
    std::cout << "cv::imread " << image_src_path.c_str() << " failed.\n";
    return 0;
  }
  cv::Mat out_image;
  if (!fisheye_undistorter_->UndistortImage(image, out_image)) {
    std::cout << "undistort failed.\n";
    return 0;
  }
  // cv::imwrite(image_dst_path, out_image);

  // int p_x = 1000;
  // int p_y = 500;
  // Eigen::Vector2d a(p_x, p_y);
  // Eigen::Vector3d b;
  // fisheye_undistorter_->p_cam_->liftProjective(a, b); //
  // 归一化平（球）面的点
  // // cv::Point2f p_un_cv = cv::Point2f(b.x() / b.z(), b.y() / b.z());
  // cv::Point2f p_un_cv = cv::Point2f(b.x(),b.y());

  // cv::Scalar red(0, 0, 255);
  // cv::circle(image, cv::Point(p_x, p_y), 5, red, 3);
  // cv::circle(out_image, p_un_cv, 5, red, 3);
  // std::cout << "b: " << b.transpose() << std::endl;
  // std::cout << "p_un_cv: " << p_un_cv << std::endl;

  // img_show_undis = image.clone();
  // showUndistortion();

  // cv::resize(image, image, cv::Size(1280, 720));
  // cv::resize(out_image, out_image, cv::Size(1280, 720));
  // cv::imshow("raw", image);
  // cv::imshow("un", out_image);
  // cv::waitKey();
  // cv::imwrite(image_dst_path, out_image);
  // cv::imwrite(image_dst_path + "raw.jpg", image);

  // FisheyeUndistort::Ptr fisheye_undistorter_2 =
  //     FisheyeUndistort::Ptr(new FisheyeUndistort(config));

  // test points
  camodocal::CataCamera::Parameters params_raw =
      fisheye_undistorter_->p_cam_->getParameters();

  Eigen::Vector3d testPoints[] = {
      Eigen::Vector3d(0, 0, 1),
      Eigen::Vector3d(1, 0, 1),
      Eigen::Vector3d(0, 1, 1),
      Eigen::Vector3d(1, 1, 1),
  };
  cv::Scalar red(0, 0, 255);
  cv::Scalar green(0, 255, 0);

  // raw image spaceToPlane
  cv::Mat img_raw = image.clone();
  for (int i = 0; i < sizeof(testPoints) / sizeof(Eigen::Vector3d); i++) {
    Eigen::Vector2d pt_new_cam;
    fisheye_undistorter_->p_cam_->spaceToPlane(testPoints[i], pt_new_cam);
    std::cout << "pt_cam with distort: " << pt_new_cam.transpose() << std::endl;
    cv::circle(img_raw, cv::Point(pt_new_cam(0), pt_new_cam(1)), 5, red, 3);
  }

  // img_undistorted_4scaled, undistort the test points
  cv::Mat remap_mat_1, remap_mat_2;
  cv::Mat K_new = fisheye_undistorter_->p_cam_->initUndistortRectifyMap(
      remap_mat_1, remap_mat_2, params_raw.gamma1() / 4.0,
      params_raw.gamma2() / 4.0, cv::Size(0, 0), params_raw.u0(),
      params_raw.v0());
  cv::Mat img_undistorted_4scaled;
  cv::remap(img_raw, img_undistorted_4scaled, remap_mat_1, remap_mat_2,
            cv::INTER_LINEAR);

  camodocal::CataCamera::Parameters param_new;
  param_new.xi() = 0;
  param_new.k1() = 0;
  param_new.k2() = 0;
  param_new.p1() = 0;
  param_new.p2() = 0;
  param_new.gamma1() = K_new.at<float>(0, 0);
  param_new.gamma2() = K_new.at<float>(1, 1);
  param_new.u0() = K_new.at<float>(0, 2);
  param_new.v0() = K_new.at<float>(1, 2);
  fisheye_undistorter_->p_cam_->setParameters(param_new);
  std::cout << "param_raw: " << params_raw << std::endl;
  std::cout << "param_new: " << param_new << std::endl;

  for (int i = 0; i < sizeof(testPoints) / sizeof(Eigen::Vector3d); i++) {
    Eigen::Vector2d pt_new_cam;
    fisheye_undistorter_->p_cam_->spaceToPlane(testPoints[i], pt_new_cam);
    std::cout << "pt_cam with distort: " << pt_new_cam.transpose() << std::endl;
    cv::circle(img_undistorted_4scaled, cv::Point(pt_new_cam(0), pt_new_cam(1)),
               5, green, 3);
  }

  cv::circle(img_undistorted_4scaled, cv::Point(50,50),
               5, green, 3);
  cv::circle(img_undistorted_4scaled, cv::Point(100,100),
               5, red, 5);

  cv::imshow("img_undistorted_4scaled", img_undistorted_4scaled);
  cv::imwrite(image_dst_path + "img_undistorted_4scaled.jpg",
              img_undistorted_4scaled);
  cv::waitKey(0);

  // cv::Mat img_undistorted_1scaled;
  // fisheye_undistorter_->p_cam_->initUndistortRectifyMap(
  //     remap_mat_1, remap_mat_2, params_raw.gamma1() / 4.0 ,
  //     params_raw.gamma2() / 4.0, cv::Size(0, 0), params_raw.u0(),
  //     params_raw.v0());

  // cv::remap(image, img_undistorted_1scaled, remap_mat_1, remap_mat_2,
  //           cv::INTER_LINEAR);

  // camodocal::CataCamera::Parameters param_new =
  //     fisheye_undistorter_->p_cam_->getParameters();
  // param_new.p1() = 0;
  // param_new.p2() = 0;
  // param_new.k1() = 0;
  // param_new.k2() = 0;

  // fisheye_undistorter_->p_cam_->setParameters(param_new);
  // for (int i = 0; i < sizeof(testPoints) / sizeof(Eigen::Vector3d); i++) {
  //     Eigen::Vector2d pt_new_cam;
  //     fisheye_undistorter_->p_cam_->spaceToPlane(testPoints[i],
  //     pt_new_cam); cv::circle(img_undistorted_1scaled,
  //                cv::Point(pt_new_cam(0), pt_new_cam(1)), 5, red, 3);
  //     std::cout << "pt_cam without distort: " << pt_new_cam.transpose()
  //               << std::endl;
  // }
  // cv::imshow("raw", img_raw);
  // cv::imshow("img_undistorted_1scaled", img_undistorted_1scaled);
  // cv::waitKey(0);
  // cv::imwrite(image_dst_path + "raw.jpg", img_raw);
  // cv::imwrite(image_dst_path + "undistorted_scale1.jpg",
  //             img_undistorted_1scaled);

  //     Eigen::Vector2d pt_raw_cam;
  //     fisheye_undistorter_2->p_cam_->spaceToPlane(testPoints[i],
  //     pt_raw_cam); cv::circle(image, cv::Point(pt_raw_cam(0),
  //     pt_raw_cam(1)),
  //                 5, red, 2, cv::LINE_AA, 4);

  // }

  // cv::imwrite(image_dst_path+"new_cam.jpg", out_image);
  // cv::imwrite(image_dst_path+"raw_cam.jpg", image);

  // std::cout << fisheye_undistorter_->p_cam_->getParameters();
  // std::cout << fisheye_undistorter_2->p_cam_->getParameters();

  // Eigen::Vector4d camera;
  // fisheye_undistorter_->readMEICameraK(camera);
  // std::cout << "camera K: " << camera << std::endl;
  // cv::Mat cv_cameraK = cv::Mat(cv::Size(2, 2), CV_64FC1);
  // cv_cameraK.at<double>(0, 0) = camera(0);
  // cv_cameraK.at<double>(0, 2) = camera(1);
  // cv_cameraK.at<double>(1, 1) = camera(2);
  // cv_cameraK.at<double>(1, 2) = camera(3);
  // std::cout << "camera K cv: " << cv_cameraK << std::endl;
  return 0;
}
