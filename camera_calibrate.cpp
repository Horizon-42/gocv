#include "camera_calibrate.h"

bool GetInternalMat(Mats pics, Size patternSize, Mat internal, Mat distCoffs)
{
    if (pics.length < 13)
    {
        return false;
    }

    // 初始化棋盘格角点的世界坐标
    std::vector<std::vector<cv::Vec3f>> objectCorners;
    int h = patternSize.height, w = patternSize.width;
    for (int i = 0; i < h; i++)
    {
        for (int j = 0; j < w; j++)
        {
            objectCorners.emplace_back(cv::Point3f(i, j, 0));
        }
    }

    // 存储所有的obejectCorners 和 imageCorners
    std::vector<std::vector<cv::Point3f>> objectPoints;
    std::vector<std::vector<cv::Point2f>> imagePoints;
    cv::Size imageSize;
    for (int i = 0; i < pics.length; i++)
    {
        // get chess boarder pic from Mats warper
        cv::Mat cbPic = *(pics.mats[i]);
        if (cbPic.empty())
        {
            continue;
        }
        else if (imageSize.empty())
        {
            imageSize.height = cbPic.size[0];
            imageSize.width = cbPic.size[1];
        }

        std::vector<cv::Point2f> imageCorners;
        bool found = cv::findChessboardCorners(cbPic, cv::Size(patternSize.width, patternSize.height), imageCorners, cv::CALIB_CB_EXHAUSTIVE | cv::CALIB_CB_ACCURACY);
        if (!found)
        {
            continue;
        }
        objectPoints.emplace_back(objectCorners);
        objectPoints.emplace_back(imageCorners);
    }
    if (objectPoints.empty())
    {
        return false;
    }
    std::vector<cv::Mat> rvecs, tvecs; //无用
    cv::calibrateCamera(objectPoints, imagePoints, imageSize, *internal, *distCoffs, rvecs, tvecs);
    // 释放内存
    std::vector<cv::Mat>().swap(rvecs);
    std::vector<cv::Mat>().swap(tvecs);
}
