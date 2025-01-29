#include "object_detector/object_detector.hpp"

namespace perception_pipeline
{
namespace object_detector
{

ObjectDetector::ObjectDetector() :
  input_6d_image_(cv::Mat())
{

}

objectDetectorErrorCode ObjectDetector::update6DImage(const cv::Mat& image)
{
  if (image.empty())
  {
    return objectDetectorErrorCode::INVALID_6D_IMAGE_ERROR;
  }

  input_6d_image_ = image;

  return objectDetectorErrorCode::OK;
}

} // namespace object_detector
} // namespace perception_pipeline

