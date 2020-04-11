#ifndef CAMERA_CALIBRATE
#define CAMERA_CALIBRATE

#ifdef __cplusplus

#include <opencv4/opencv2/calib3d.hpp>
#include <opencv4/opencv2/opencv.hpp>

extern "C"
{
#endif
#include "core.h"

    bool GetInternalMat(Mats pics, Size patternSize, Mat internal, Mat distCoffs);
    Mat GetExternalMat(Mats pics, Size patternSize);

#ifdef __cplusplus
}
#endif

#endif // CAMERA_CALIBRATE