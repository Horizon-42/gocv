#include "calib3d.h"

void Fisheye_UndistortImage(Mat distorted, Mat undistorted, Mat k, Mat d)
{
    cv::fisheye::undistortImage(*distorted, *undistorted, *k, *d);
}

void Fisheye_UndistortImageWithParams(Mat distorted, Mat undistorted, Mat k, Mat d, Mat knew, Size size)
{
    cv::Size sz(size.width, size.height);
    cv::fisheye::undistortImage(*distorted, *undistorted, *k, *d, *knew, sz);
}

void InitUndistortRectifyMap(Mat cameraMatrix, Mat distCoeffs, Mat r, Mat newCameraMatrix, Size size, int m1type, Mat map1, Mat map2)
{
    cv::Size sz(size.width, size.height);
    cv::initUndistortRectifyMap(*cameraMatrix, *distCoeffs, *r, *newCameraMatrix, sz, m1type, *map1, *map2);
}

Mat GetOptimalNewCameraMatrixWithParams(Mat cameraMatrix, Mat distCoeffs, Size size, double alpha, Size newImgSize, Rect *validPixROI, bool centerPrincipalPoint)
{
    cv::Size sz(size.width, size.height);
    cv::Size newSize(newImgSize.width, newImgSize.height);
    cv::Rect rect(validPixROI->x, validPixROI->y, validPixROI->width, validPixROI->height);
    cv::Mat *mat = new cv::Mat(cv::getOptimalNewCameraMatrix(*cameraMatrix, *distCoeffs, sz, alpha, newSize, &rect, centerPrincipalPoint));
    validPixROI->x = rect.x;
    validPixROI->y = rect.y;
    validPixROI->width = rect.width;
    validPixROI->height = rect.height;
    return mat;
}

void Undistort(Mat src, Mat dst, Mat cameraMatrix, Mat distCoeffs, Mat newCameraMatrix)
{
    cv::undistort(*src, *dst, *cameraMatrix, *distCoeffs, *newCameraMatrix);
}

bool FindChessboardCorners(Mat image, Size patternSize, Mat corners, int flags)
{
    cv::Size sz(patternSize.width, patternSize.height);
    return cv::findChessboardCorners(*image, sz, *corners, flags);
}

bool FindChessboardCornersSB(Mat image, Size patternSize, Mat corners, int flags)
{
    cv::Size sz(patternSize.width, patternSize.height);
    return cv::findChessboardCornersSB(*image, sz, *corners, flags);
}

void DrawChessboardCorners(Mat image, Size patternSize, Mat corners, bool patternWasFound)
{
    cv::Size sz(patternSize.width, patternSize.height);
    cv::drawChessboardCorners(*image, sz, *corners, patternWasFound);
}

double CalibrateCamera(Mat objectPoints, Mats imagePoints, Size imageSize, Mat cameraMatrix, Mat distCoeffs,
                       int flags, TermCriteria criteria)
{
    cv::Size imsz(imageSize.width, imageSize.height);
    std::vector<cv::Mat> rotateVecs, transVecs;
    std::vector<std::vector<cv::Point3f>> objectPointsVec;
    std::vector<std::vector<cv::Point2f>> imagePointsVec;
    std::vector<cv::Point3f> objPts;
    for (int i = 0; i < objectPoints->rows; i++)
    {
        auto row = objectPoints->ptr<float>(i);
        objPts.emplace_back(cv::Point3f(row[0], row[1], row[2]));
    }

    for (int i = 0; i < imagePoints.length; i++)
    {
        objectPointsVec.push_back(objPts);
        std::vector<cv::Point2f> imgPts;
        std::cout << *(imagePoints.mats[i]) << std::endl;
        for (int j = 0; j < imagePoints.mats[i]->rows; j++)
        {
            auto row = imagePoints.mats[i]->ptr<float>(i);
            imgPts.emplace_back(cv::Point2f(row[0], row[1]));
        }

        imagePointsVec.push_back(imgPts);
    }
    std::cout << "calibrate..." << std::endl;
    std::cout << imsz << std::endl;
    std::cout << objectPointsVec.size() << "\t" << imagePointsVec.size() << std::endl;
    double ret = cv::calibrateCamera(objectPointsVec, imagePointsVec, imsz, *cameraMatrix, *distCoeffs, rotateVecs, transVecs, cv::CALIB_FIX_K3 + cv::CALIB_FIX_K4 + cv::CALIB_FIX_K5);
    std::cout << "done" << std::endl;
    //    // 给 rvecs 和 tvecs 赋值
    //    rvecs->mats = new Mat[rotateVecs.size()];
    //    tvecs->mats = new Mat[transVecs.size()];
    //    for (int i = 0; i < rotateVecs.size(); i++)
    //    {
    //        rvecs->mats[i] = new cv::Mat(rotateVecs[i]);
    //        tvecs->mats[i] = new cv::Mat(transVecs[i]);
    //    }
    //
    //    rvecs->length = int(rotateVecs.size());
    //    tvecs->length = int(transVecs.size());

    return ret;
}