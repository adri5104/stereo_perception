#ifndef OBJECT_DETECTOR_HPP
#define OBJECT_DETECTOR_HPP

#include "object_detector/world_entity.hpp"
#include <opencv2/opencv.hpp>

namespace perception_pipeline
{
namespace object_detector
{


// Enum class for error codes
enum class objectDetectorErrorCode {
    OK = 1,
    INVALID_6D_IMAGE_ERROR,
};

// Helper function to get error messages
/**
 * @brief Get the error message for a given WorldPointErrorCode
 * 
 * @param code The error code for which the message is required
 * @return std::string The error message corresponding to the error code
 */
inline std::string getErrorMessageObjectDetector(objectDetectorErrorCode code) {
    static const std::unordered_map<objectDetectorErrorCode, std::string> errorMessages = {
        {objectDetectorErrorCode::OK, "No error."},
        {objectDetectorErrorCode::INVALID_6D_IMAGE_ERROR, "Invalid 6D image."}
    };
   auto it = errorMessages.find(code);
    return it != errorMessages.end() 
        ? it->second 
        : "Invalid error code.";
}

class ObjectDetector
{
  public: 
    /**
     * @brief Construct a new Object Detector object
     * 
     */
    ObjectDetector();

    /**
     * @brief Update the 6D image with a new image
     * 
     * @param image The new 6D image
     * @return objectDetectorErrorCode The error code for the operation
     */
    objectDetectorErrorCode update6DImage(const cv::Mat&);
    
  
  private:

    cv::Mat input_6d_image_; // The current 6D image
    
    
};

} // namespace object_detector
} // namespace perception_pipeline


#endif // OBJECT_DETECTOR_HPP