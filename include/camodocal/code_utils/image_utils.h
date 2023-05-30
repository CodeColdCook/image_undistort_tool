#ifndef IMAGE_UTLIS_H
#define IMAGE_UTLIS_H

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

bool try_load_image(const std::string &file_name, cv::Mat &img);

bool check_for_save(const std::string &path, cv::Mat *image_src,
                    cv::Mat *image_dst, bool print_tips);

#endif