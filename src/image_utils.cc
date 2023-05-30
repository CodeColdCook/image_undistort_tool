#include "camodocal/code_utils/image_utils.h"

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