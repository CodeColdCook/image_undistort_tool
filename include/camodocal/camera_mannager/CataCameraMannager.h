#ifndef CATACAMERAMANNAGER_H
#define CATACAMERAMANNAGER_H

#include "CameraMannager.h"
#include "camodocal/camera_models/CataCamera.h"

namespace camodocal {

class CataCameraMannager : public CameraMannager {
 public:
  CataCameraMannager(const std::string &config_path, const float &scale)
      : CameraMannager(config_path, scale){
    Init();
  };

  camodocal::CataCameraPtr camera_raw_ptr_;
  camodocal::CataCameraPtr camera_scaled_ptr_;

  camodocal::CataCamera::Parameters params_raw_;
  camodocal::CataCamera::Parameters params_scaled_;

  bool Init() override;
};
typedef boost::shared_ptr<CataCameraMannager> CataCameraMannagerPtr;
typedef boost::shared_ptr<const CataCameraMannager> CataCameraMannagerConstPtr;
}  // namespace camodocal
#endif