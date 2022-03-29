// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#ifndef MONO2D_DET_IMAGE_UTILS_H
#define MONO2D_DET_IMAGE_UTILS_H

#include <memory>
#include <string>
#include <vector>

#include "dnn_node/dnn_node.h"
#include "opencv2/core/mat.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"

using hobot::dnn_node::NV12PyramidInput;

#define ALIGNED_2E(w, alignment) \
  ((static_cast<uint32_t>(w) + (alignment - 1U)) & (~(alignment - 1U)))
#define ALIGN_4(w) ALIGNED_2E(w, 4U)
#define ALIGN_8(w) ALIGNED_2E(w, 8U)
#define ALIGN_16(w) ALIGNED_2E(w, 16U)
#define ALIGN_64(w) ALIGNED_2E(w, 64U)

class ImageUtils {
 public:
  static std::shared_ptr<NV12PyramidInput> GetNV12Pyramid(
      const cv::Mat &image,
      int scaled_img_height,
      int scaled_img_width);

  // 输入图片size小于scale size（模型输入size）：将输入图片padding到左上区域
  // 输入图片size大于scale size（模型输入size）：crop输入图片左上区域
  static std::shared_ptr<NV12PyramidInput> GetNV12PyramidFromNV12Img(
      const char* in_img_data,
      int in_img_height,
      int in_img_width,
      int scaled_img_height,
      int scaled_img_width);

  static int32_t BGRToNv12(cv::Mat &bgr_mat, cv::Mat &img_nv12);
};

#endif  // MONO2D_DET_IMAGE_UTILS_H
