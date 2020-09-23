#ifndef CAMERA_CALIBRATE
#define CAMERA_CALIBRATE

#ifdef __cplusplus

#include <opencv4/opencv2/calib3d.hpp>
#include <opencv4/opencv2/opencv.hpp>

bool GetExternalMat(const cv::Mat &pic, const cv::Mat &cameraMatrix,
                    const cv::Mat &distCoffs, cv::Size patternSize,
                    cv::Mat &external);

extern "C" {
#endif
#include "core.h"

double GetInternalMat(Mats pics, Size patternSize, Mat cameraMatrix,
                      Mat distCoffs, bool accuracy);
bool GetBMat(Mats pics, Mats cameraMatrix, Mats distCoffs, Size patternSize,
             Mat B);

bool GetStereoBMat();

bool GetEMat(Mat pic, Mat cameraMatrix, Mat distCoffs, Size patternSize,
             Mat externalMat);

bool ProjectPoints(Mat objectPoints, Mat externalMat, Mat cameraMatrix,
                   Mat distCoeffs, Mat imagePoints);

#ifdef __cplusplus
}
#endif

#endif // CAMERA_CALIBRATE