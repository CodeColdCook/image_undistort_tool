#ifndef PINHOLECAMERAMANNAGER_H
#define PINHOLECAMERAMANNAGER_H

#include "CameraMannager.h"
#include "camodocal/camera_models/PinholeCamera.h"

namespace camodocal {

class PinholeCameraMannager : public CameraMannager {
 public:
  PinholeCameraMannager(const std::string &config_path, const float &scale)
      : CameraMannager(config_path, scale){
    Init();
  };

  camodocal::PinholeCameraPtr camera_raw_ptr_;
  camodocal::PinholeCameraPtr camera_scaled_ptr_;

  camodocal::PinholeCamera::Parameters params_raw_;
  camodocal::PinholeCamera::Parameters params_scaled_;

  bool Init() override;
};

typedef boost::shared_ptr<PinholeCameraMannager> PinholeCameraMannagerPtr;
typedef boost::shared_ptr<const PinholeCameraMannager> PinholeCameraMannagerConstPtr;

}  // namespace camodocal
#endif