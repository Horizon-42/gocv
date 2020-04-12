#include "calibrate3d.h"

int CalibrateCamera(Mat objectPoints, Mat imagePoints, Size imageSize, Mat cameraMatrix, Mat distCoeffs, Mats *rvecs, Mats *tvecs, int flags, TermCriteria criteria)
{
    cv::Size imsz(imageSize.width, imageSize.height);
    std::vector<cv::Mat> rotateVecs;
    std::vector<cv::Mat> transVecs;
    int ret = cv::calibrateCamera(*objectPoints, *imagePoints, imsz, *cameraMatrix, *distCoeffs, rotateVecs, transVecs, flags, *criteria);
    // 给 rvecs 和 tvecs 赋值
    rvecs->mats = new Mat[rotateVecs.size()];
    tvecs->mats = new Mat[transVecs.size()];
    for (int i = 0; i < rotateVecs.size(); i++)
    {
        rvecs->mats[i] = new cv::Mat(rotateVecs[i]);
        tvecs->mats[i] = new cv::Mat(transVecs[i]);
    }

    rvecs->length = int(rotateVecs.size());
    tvecs->length = int(transVecs.size());

    return ret;
}
