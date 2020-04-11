#ifndef CAMERA_CALIBRATE
#define CAMERA_CALIBRATE

#ifdef __cplusplus

#include <opencv4/opencv2/calib3d.hpp>
#include <opencv4/opencv2/opencv.hpp>

bool GetExternalMat(cv::Mat pic, cv::Mat cameraMatrix, cv::Mat distCoffs, cv::Size patternSize, cv::Mat &external);

extern "C"
{
#endif
#include "core.h"

    double GetInternalMat(Mats pics, Size patternSize, Mat internal, Mat distCoffs);
    bool CalculateBMat(Mats pics, Mats cameraMatrixs, Mats distCoffs, Size patternSize, Mat B);

#ifdef __cplusplus
}
#endif

#endif // CAMERA_CALIBRATE