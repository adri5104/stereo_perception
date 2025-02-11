#ifndef _OBJECT_DETECTOR_ERROR_CODES_HPP_
#define _OBJECT_DETECTOR_ERROR_CODES_HPP_

#include <unordered_map>
#include <cmath>
#include <string>

namespace perception_pipeline
{
namespace object_detector
{

/**
 * @brief Error codes for ObjectDetector operations.
 */
enum class objectDetectorErrorCode {
    OK = 1,                    ///< Operation successful.
    INVALID_6D_IMAGE_ERROR,     ///< Input 6D image is invalid or empty.
};
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

}
}

#endif // _OBJECT_DETECTOR_ERROR_CODES_HPP_