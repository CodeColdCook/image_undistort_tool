# Introduction

多相机模型去畸变工具
part of [camodocal](https://github.com/hengli/camodocal)

## Dependence

- [OpenCV](https://github.com/opencv/opencv) : 3.4,4.X
- [ceres-solver](<http://ceres-solver.org>) : 2.0.0

## Usage

- 支持相机模型：
  - mei
  - pinhole

### Undistortion

See [CameraMannager.h](include/camodocal/camera_mannager/CameraMannager.h) for general interface, See [cata_undistortor.cc](test/common/cata_undistortor.cc) for general useage.
