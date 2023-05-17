#pragma once
#include <experimental/filesystem>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"

#ifndef DEG_TO_RAD
#define DEG_TO_RAD (M_PI / 180.0)
#endif

struct FisheyeUndistortconfig {
    int imgWidth = 1920;
    int imgHeight = 1080;
    double fov_camera = 134;
    double fov_center_roi = 120;
    Eigen::Vector3d cameraRotation;
    std::string camFilePath;
    double scale;
};

struct MEIConfig {
    double xi;
    double k1;
    double k2;
    double p1;
    double p2;
    double gamma1;
    double gamma2;
    double u0;
    double v0;
};

class FisheyeUndistort {
   public:
    // camodocal::CameraPtr p_cam_;
    camodocal::CataCameraPtr p_cam_;

   private:
    bool inited_;
    FisheyeUndistortconfig config_;
    cv::Mat undistMap_;
    Eigen::AngleAxisd roi_direction_;

    // remap method2
    cv::Mat map1_, map2_;

   public:
    typedef boost::shared_ptr<FisheyeUndistort> Ptr;
    FisheyeUndistort(FisheyeUndistortconfig config)
        : inited_(false), config_(config) {
        inited_ = Init();
    }
    // ~FisheyeUndistort() = default;

    inline cv::Mat get_undist_map() { return undistMap_; }

    inline std::string get_cam_file_path() { return config_.camFilePath; }

    bool UndistortImage(const cv::Mat img_src, cv::Mat &img_dst) {
        if (!inited_) {
            printf("FisheyeUndistort init failed ...");
            return false;
        }

        if (img_src.empty()) {
            printf("Undistort failed, image is empty ...");
            return false;
        }
        // cv::remap(img_src, img_dst, undistMap_, cv::Mat(), cv::INTER_LINEAR);
        cv::remap(img_src, img_dst, map1_, map2_, cv::INTER_LINEAR);
        if (img_dst.empty()) {
            printf("Undistort failed, result error ...");
            return false;
        }
        return true;
    }

    void Reset(FisheyeUndistortconfig config) {
        config_ = config;
        inited_ = Init();
    }

    bool readMEIFromYamlFile(MEIConfig &mei_cfg) {
        cv::FileStorage fs(config_.camFilePath, cv::FileStorage::READ);

        if (!fs.isOpened()) {
            return false;
        }

        if (!fs["model_type"].isNone()) {
            std::string sModelType;
            fs["model_type"] >> sModelType;

            if (sModelType.compare("MEI") != 0) {
                return false;
            }
        }

        cv::FileNode n = fs["mirror_parameters"];
        mei_cfg.xi = static_cast<double>(n["xi"]);

        n = fs["distortion_parameters"];
        mei_cfg.k1 = static_cast<double>(n["k1"]);
        mei_cfg.k2 = static_cast<double>(n["k2"]);
        mei_cfg.p1 = static_cast<double>(n["p1"]);
        mei_cfg.p2 = static_cast<double>(n["p2"]);

        n = fs["projection_parameters"];
        mei_cfg.gamma1 = static_cast<double>(n["gamma1"]);
        mei_cfg.gamma2 = static_cast<double>(n["gamma2"]);
        mei_cfg.u0 = static_cast<double>(n["u0"]);
        mei_cfg.v0 = static_cast<double>(n["v0"]);

        return true;
    }

    bool readMEICameraK(Eigen::Vector4d &camK) {
        MEIConfig mei_cfg;
        if (!readMEIFromYamlFile(mei_cfg)) {
            return false;
        }
        // double inv_K11 = 1.0 / mei_cfg.gamma1;
        // double inv_K13 = -mei_cfg.u0 / mei_cfg.gamma1;
        // double inv_K22 = 1.0 / mei_cfg.gamma2;
        // double inv_K23 = -mei_cfg.v0 / mei_cfg.gamma2;
        camK(0) = mei_cfg.gamma1;
        camK(1) = mei_cfg.u0;
        camK(2) = mei_cfg.gamma2;
        camK(3) = mei_cfg.v0;
        return true;
    }

//    private:
    bool Init() {
        if (!std::experimental::filesystem::exists(config_.camFilePath)) {
            printf("Camera file does not exist, %s \n",
                   config_.camFilePath.c_str());
            return false;
        }
        if (config_.imgWidth <= 0) {
            printf("Resolution must be non-negative \n");
            return false;
        }
        if (config_.fov_camera < 0 || config_.fov_center_roi < 0) {
            printf("FOV must be non-negative \n");
            return false;
        }

        // p_cam_ =
        //     camodocal::CameraFactory::instance()->generateCameraFromYamlFile(
        //         config_.camFilePath);
        p_cam_ = camodocal::CataCameraPtr(new camodocal::CataCamera);
        camodocal::CataCamera::Parameters params = p_cam_->getParameters();
        params.readFromYamlFile(config_.camFilePath);
        p_cam_->setParameters(params);

        // double fov_center = (double)config_.imgWidth / 2.0 /
        //                     tan(config_.fov_center_roi * DEG_TO_RAD / 2);
        // roi_direction_ =
        //     Eigen::AngleAxis<double>(config_.cameraRotation.norm(),
        //                              config_.cameraRotation.normalized())
        //         .inverse();
        // MakeUndistMap(fov_center);

        // MakeUndistMap2();
        return true;
    }
    /// @brief
    /// @return
    void MakeUndistMap(const double &fov_center) {
        undistMap_ = cv::Mat(config_.imgHeight, config_.imgWidth, CV_32FC2);
        printf("Generating map of size (%d,%d)  \n", undistMap_.size[0],
               undistMap_.size[1]);
        printf("Perspective facing (%.2f,%.2f,%.2f)  \n",
               roi_direction_._transformVector(Eigen::Vector3d(0, 0, 1))[0],
               roi_direction_._transformVector(Eigen::Vector3d(0, 0, 1))[1],
               roi_direction_._transformVector(Eigen::Vector3d(0, 0, 1))[2]);
        for (int x = 0; x < config_.imgWidth; x++)
            for (int y = 0; y < config_.imgHeight; y++) {
                Eigen::Vector3d objPoint =
                    roi_direction_ *
                    Eigen::Vector3d(((double)x - (double)config_.imgWidth / 2),
                                    ((double)y - (double)config_.imgHeight / 2),
                                    fov_center);
                Eigen::Vector2d imgPoint;
                p_cam_->spaceToPlane(objPoint, imgPoint);
                undistMap_.at<cv::Vec2f>(cv::Point(x, y)) =
                    cv::Vec2f(imgPoint.x(), imgPoint.y());
            }

        printf("Upper corners: (%.2f, %.2f), (%.2f, %.2f)  \n",
               undistMap_.at<cv::Vec2f>(cv::Point(0, 0))[0],
               undistMap_.at<cv::Vec2f>(cv::Point(0, 0))[1],
               undistMap_.at<cv::Vec2f>(cv::Point(config_.imgWidth - 1, 0))[0],
               undistMap_.at<cv::Vec2f>(cv::Point(config_.imgWidth - 1, 0))[1]);
    }

    void MakeUndistMap2() {
        camodocal::CataCamera::Parameters param_new = p_cam_->getParameters();
        // param_new.gamma1() /= config_.scale;
        // param_new.gamma2() /= config_.scale;
        cv::Mat K_test = p_cam_->initUndistortRectifyMap(
            map1_, map2_, param_new.gamma1(), param_new.gamma2(),
            cv::Size(0, 0), param_new.u0(), param_new.v0());
        //  std::cout << "*** K_scaled: " << K_test << std::endl;

        // p_cam_->initUndistortMap(map1_, map2_, config_.scale);
       
        // param_new.k1() = 0;
        // param_new.k2() = 0;
        // param_new.p1() = 0;
        // param_new.p2() = 0;
        // p_cam_->setParameters(param_new);
    }
};
