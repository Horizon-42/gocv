#ifndef _OPENCV4_CALIBRATE_H_
#define _OPENCV4_CALIBRATE_H_

#ifdef __cplusplus
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <vector>

extern "C"
{
#endif

#include "core.h"

    // Calib
    int CalibrateCamera(Mat objectPoints, Mat imagePoints, Size imageSize, Mat cameraMatrix, Mat distCoeffs, Mats rvecs, Mats tvecs, int flags, TermCriteria criteria);

#ifdef __cplusplus
}
#endif

#endif //_OPENCV4_CALIBRATE_H_