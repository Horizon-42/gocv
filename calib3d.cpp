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
    //    无用
    std::vector<cv::Mat> rotateVecs, transVecs;
    std::vector<cv::Mat> objectPointsVec;
    std::vector<cv::Mat> imagePointsVec;
    for (int i = 0; i < imagePoints.length; i++)
    {
        objectPointsVec.push_back(*objectPoints);
        imagePointsVec.push_back(*(imagePoints.mats[i]));
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

double StereoCalibrate(Mat objectCorners, Mat imagePoints1, Mat imagePoints2, Mat cameraMatrix1, Mat distCoeffs1, Mat cameraMatrix2, Mat distCoeffs2,
                       Size imageSize, Mat R, Mat T, Mat E, Mat F, Mat img1, Mat img2, Mat canvas)
{
    std::cout << "StereoCalibrate" << std::endl;
    cv::Size imsz(imageSize.width, imageSize.height);
    std::vector<cv::Mat> objectPointsVec;
    std::vector<cv::Mat> imagePointsVec1, imagePointsVec2;
    objectPointsVec.emplace_back(*objectCorners);
    imagePointsVec1.emplace_back(*imagePoints1);
    imagePointsVec2.emplace_back(*imagePoints2);

    double rms = cv::stereoCalibrate(objectPointsVec, imagePointsVec1, imagePointsVec2,
                                     *cameraMatrix1, *distCoeffs1,
                                     *cameraMatrix2, *distCoeffs2,
                                     imsz, *R, *T, *E, *F,
                                     cv::CALIB_FIX_ASPECT_RATIO +
                                         cv::CALIB_ZERO_TANGENT_DIST +
                                         cv::CALIB_USE_INTRINSIC_GUESS +
                                         cv::CALIB_SAME_FOCAL_LENGTH +
                                         cv::CALIB_RATIONAL_MODEL +
                                         cv::CALIB_FIX_K3 + cv::CALIB_FIX_K4 + cv::CALIB_FIX_K5,
                                     cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, 1e-5));

    std::cout << "done with RMS error=" << rms << std::endl;

    double err = 0;
    int npoints = imagePoints1->size[0] * 2;
    cv::Mat imgpt[2]{*imagePoints1, *imagePoints2};
    cv::Mat cameraMatrix[2]{*cameraMatrix1, *cameraMatrix2};
    cv::Mat distCoeffs[2]{*distCoeffs1, *distCoeffs2};
    std::vector<cv::Vec3f> lines[2];
    for (int k = 0; k < 2; k++)
    {
        undistortPoints(imgpt[k], imgpt[k], cameraMatrix[k], distCoeffs[k], cv::Mat(), cameraMatrix[k]);
        computeCorrespondEpilines(imgpt[k], k + 1, *F, lines[k]);
    }
    for (int j = 0; j < imagePoints1->size[0]; j++)
    {
        double errij = fabs(imgpt[0].at<cv::Vec2f>(j, 0)[0] * lines[1][j][0] +
                            imgpt[0].at<cv::Vec2f>(j, 0)[1] * lines[1][j][1] + lines[1][j][2]) +
                       fabs(imgpt[1].at<cv::Vec2f>(j, 0)[0] * lines[0][j][0] +
                            imgpt[1].at<cv::Vec2f>(j, 0)[1] * lines[0][j][1] + lines[0][j][2]);
        err += errij;
    }
    std::cout << "average epipolar err = " << err / npoints << std::endl;

    cv::Mat R1, R2, P1, P2, Q;
    cv::Rect validRoi[2];

    stereoRectify(cameraMatrix[0], distCoeffs[0],
                  cameraMatrix[1], distCoeffs[1],
                  imsz, *R, *T, R1, R2, P1, P2, Q,
                  cv::CALIB_ZERO_DISPARITY, 0, imsz, &validRoi[0], &validRoi[1]);

    // OpenCV can handle left-right
    // or up-down camera arrangements
    bool isVerticalStereo = fabs(P2.at<double>(1, 3)) > fabs(P2.at<double>(0, 3));

    cv::Mat H1, H2;
    stereoRectifyUncalibrated(imgpt[0], imgpt[1], *F, imsz, H1, H2, 3);

    R1 = cameraMatrix[0].inv() * H1 * cameraMatrix[0];
    R2 = cameraMatrix[1].inv() * H2 * cameraMatrix[1];
    P1 = cameraMatrix[0];
    P2 = cameraMatrix[1];

    //Precompute maps for cv::remap()
    cv::Mat rmap[2][2];
    initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1, imsz, CV_16SC2, rmap[0][0], rmap[0][1]);
    initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, imsz, CV_16SC2, rmap[1][0], rmap[1][1]);

    double sf;
    int w, h;
    if (!isVerticalStereo)
    {
        sf = 600. / MAX(imageSize.width, imageSize.height);
        w = cvRound(imageSize.width * sf);
        h = cvRound(imageSize.height * sf);
        (*canvas).create(h, w * 2, CV_8UC3);
    }
    else
    {
        sf = 300. / MAX(imageSize.width, imageSize.height);
        w = cvRound(imageSize.width * sf);
        h = cvRound(imageSize.height * sf);
        (*canvas).create(h * 2, w, CV_8UC3);
    }

    cv::Mat imgs[2]{*img1, *img2};
    for (int k = 0; k < 2; k++)
    {
        cv::Mat cimg;
        remap(imgs[k], cimg, rmap[k][0], rmap[k][1], cv::INTER_LINEAR);

        cv::Mat canvasPart = !isVerticalStereo ? (*canvas)(cv::Rect(w * k, 0, w, h)) : (*canvas)(cv::Rect(0, h * k, w, h));
        resize(cimg, canvasPart, canvasPart.size(), 0, 0, cv::INTER_AREA);

        cv::Rect vroi(cvRound(validRoi[k].x * sf), cvRound(validRoi[k].y * sf),
                      cvRound(validRoi[k].width * sf), cvRound(validRoi[k].height * sf));
        cv::rectangle(canvasPart, vroi, cv::Scalar(0, 0, 255), 3, 8);
    }

    if (!isVerticalStereo)
    {
        for (int j = 0; j < canvas->rows; j += 16)
            line(*canvas, cv::Point(0, j), cv::Point(canvas->cols, j), cv::Scalar(0, 255, 0), 1, 8);
    }
    else
        for (int j = 0; j < canvas->cols; j += 16)
            line(*canvas, cv::Point(j, 0), cv::Point(j, canvas->rows), cv::Scalar(0, 255, 0), 1, 8);

    return rms;
}
