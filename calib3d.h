#ifndef _OPENCV3_CALIB_H_
#define _OPENCV3_CALIB_H_

#ifdef __cplusplus
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

extern "C"
{
#endif

#include "core.h"

    //Calib
    void Fisheye_UndistortImage(Mat distorted, Mat undistorted, Mat k, Mat d);
    void Fisheye_UndistortImageWithParams(Mat distorted, Mat undistorted, Mat k, Mat d, Mat knew, Size size);

    void InitUndistortRectifyMap(Mat cameraMatrix, Mat distCoeffs, Mat r, Mat newCameraMatrix, Size size, int m1type, Mat map1, Mat map2);
    Mat GetOptimalNewCameraMatrixWithParams(Mat cameraMatrix, Mat distCoeffs, Size size, double alpha, Size newImgSize, Rect *validPixROI, bool centerPrincipalPoint);
    void Undistort(Mat src, Mat dst, Mat cameraMatrix, Mat distCoeffs, Mat newCameraMatrix);

    bool FindChessboardCornersSB(Mat image, Size patternSize, Mat corners, int flags);
    bool FindChessboardCorners(Mat image, Size patternSize, Mat corners, int flags);
    void DrawChessboardCorners(Mat image, Size patternSize, Mat corners, bool patternWasFound);
    double CalibrateCamera(Mat objectPoints, Mats imagePoints, Size imageSize, Mat cameraMatrix, Mat distCoeffs, int flags, TermCriteria criteria);
    double StereoCalibrate(Mat objectCorners, Mat imagePoints1, Mat imagePoints2, Mat cameraMatrix1, Mat distCoeffs1, Mat cameraMatrix2, Mat distCoeffs2,
                           Size imageSize, Mat R, Mat T, Mat E, Mat F);

#ifdef __cplusplus
}
#endif

#endif //_OPENCV3_CALIB_H
