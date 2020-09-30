#include "camera_calibrate.h"

double GetInternalMat(Mats pics, Size patternSize, Mat cameraMatrix,
                      Mat distCoffs, bool accurcy)
{
  if (pics.length < 3)
  {
    return -1;
  }
  // 初始化棋盘格角点的世界坐标
  std::vector<cv::Point3f> objectCorners;
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
    //        cv::imshow("cbPic",cbPic);
    //        cv::waitKey(0);
    if (cbPic.empty())
    {
      continue;
    }
    if (imageSize.empty())
    {
      imageSize.height = cbPic.size[0];
      imageSize.width = cbPic.size[1];
    }
    std::vector<cv::Point2f> imageCorners;

    if (accurcy)
    {
      bool found = cv::findChessboardCornersSB(
          cbPic, cv::Size(patternSize.width, patternSize.height), imageCorners,
          cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_ACCURACY);
      if (!found)
      {
        continue;
      }
    }
    else
    {
      bool found = cv::findChessboardCorners(
          cbPic, cv::Size(patternSize.width, patternSize.height), imageCorners,
          cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE +
              cv::CALIB_CB_FAST_CHECK + cv::CALIB_CB_ACCURACY);
      if (!found)
      {
        continue;
      }
      cv::cornerSubPix(
          cbPic, imageCorners, cv::Size(5, 5), cv::Size(-1, -1),
          cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER,
                           40, 0.001));
    }

    objectPoints.emplace_back(objectCorners);
    imagePoints.emplace_back(imageCorners);
    cbPic.release();
    std::vector<cv::Point2f>().swap(imageCorners);
  }
  if (objectPoints.empty())
  {
    return -1;
  }
  std::vector<cv::Mat> rvecs, tvecs; //无用
  double res = cv::calibrateCamera(objectPoints, imagePoints, imageSize,
                                   *cameraMatrix, *distCoffs, rvecs, tvecs,
                                   cv::CALIB_FIX_K3 + cv::CALIB_FIX_K4 + cv::CALIB_FIX_K5);

  // 释放内存
  std::vector<cv::Mat>().swap(rvecs);
  std::vector<cv::Mat>().swap(tvecs);
  return res;
}

bool GetExternalMat(const cv::Mat &pic, const cv::Mat &cameraMatrix,
                    const cv::Mat &distCoffs, cv::Size patternSize,
                    cv::Mat &external)
{
  if (pic.empty() || cameraMatrix.empty() || distCoffs.empty())
  {
    return false;
  }

  // 初始化棋盘格角点的世界坐标
  std::vector<cv::Point3f> objectCorners;
  int h = patternSize.height, w = patternSize.width;
  for (int i = 0; i < h; i++)
  {
    for (int j = 0; j < w; j++)
    {
      objectCorners.emplace_back(cv::Point3f(i, j, 0));
    }
  }

  std::vector<cv::Point2f> imageCorners;
  bool found = cv::findChessboardCorners(
      pic, cv::Size(patternSize.width, patternSize.height), imageCorners,
      cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_ACCURACY |
          cv::CALIB_CB_EXHAUSTIVE);
  if (!found)
  {
    return false;
  }
  cv::cornerSubPix(
      pic, imageCorners, cv::Size(5, 5), cv::Size(-1, -1),
      cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 40,
                       0.001));

  cv::Mat rvec, tvec;
  cv::solvePnPRansac(objectCorners, imageCorners, cameraMatrix, distCoffs, rvec,
                     tvec);
  // 根据旋转、平移矩阵构造外参矩阵
  external = cv::Mat::zeros(cv::Size(4, 4), CV_64FC1);
  cv::Mat rotateMat;
  cv::Rodrigues(rvec, rotateMat);

  for (int i = 0; i < 3; i++)
  {
    auto rvecRow = rotateMat.ptr<double>(i);
    auto tvecRow = tvec.ptr<double>(i);
    auto exRow = external.ptr<double>(i);
    for (int j = 0; j < 3; j++)
    {
      exRow[j] = rvecRow[j];
    }
    exRow[3] = tvecRow[0];
  }
  external.at<double>(3, 3) = 1;

  // 释放内存
  rvec.release();
  tvec.release();
  rotateMat.release();
  std::vector<cv::Point3f>().swap(objectCorners);
  std::vector<cv::Point2f>().swap(imageCorners);
  return true;
}

bool GetBMat(Mats pics, Mats cameraMatrixs, Mats distCoffs, Size patternSize,
             Mat B)
{
  if (pics.length != 2 || cameraMatrixs.length != 2 || distCoffs.length != 2)
  {
    return false;
  }
  cv::Mat picI = *(pics.mats[0]);
  cv::Mat picJ = *(pics.mats[1]);
  if (picI.empty() || picJ.empty())
  {
    return false;
  }
  cv::Mat cmI = *(cameraMatrixs.mats[0]);
  cv::Mat cmJ = *(cameraMatrixs.mats[1]);
  if (cmI.empty() || cmJ.empty())
  {
    return false;
  }

  cv::Mat disI = *(distCoffs.mats[0]);
  cv::Mat disJ = *(distCoffs.mats[1]);
  if (disI.empty() || disJ.empty())
  {
    return false;
  }

  cv::Size psz(patternSize.width, patternSize.height);
  cv::Mat eI, eJ;

  if (!GetExternalMat(picI, cmI, disI, psz, eI) ||
      !GetExternalMat(picJ, cmJ, disJ, psz, eJ))
  {
    return false;
  }

  //  std::cout<<"output bji mats"<<std::endl;
  //
  //  cv::Mat Rj = eJ(cv::Rect(0,0,3,3)).clone();
  //  cv::Mat Tj = eJ(cv::Rect(3,0,1,3)).clone();
  //  std::cout<<Rj<<std::endl;
  //  std::cout<<"---------------"<<std::endl;
  //  std::cout<<Tj<<std::endl;
  //  std::cout<<"---------------"<<std::endl;
  //
  //  std::cout<<eJ<<std::endl;
  //  std::cout<<"---------------"<<std::endl;
  //
  //
  //  cv::Mat Ri = eI(cv::Rect(0,0,3,3)).clone();
  //  cv::Mat Ti = eI(cv::Rect(3,0,1,3)).clone();
  //  std::cout<<Ri<<std::endl;
  //  std::cout<<"---------------"<<std::endl;
  //
  //  std::cout<<Ti<<std::endl;
  //  std::cout<<"---------------"<<std::endl;
  //  std::cout<<eI<<std::endl;
  //
  //   std::cout<<"&&&&&&&&&&&&&&&&&&&"<<std::endl;
  //  cv::Mat RiInv;
  //  cv::invert(Ri,RiInv);
  //  cv::Mat Rji = Rj*RiInv;
  //  std::cout<<Rji<<std::endl<<std::endl;
  //  cv::Mat Tji = Tj-Rji*Ti;
  //  std::cout<<Tji<<std::endl;
  //  std::cout<<"&&&&&&&&&&&&&&&&&&&"<<std::endl;

  //  Ej = Bji*Ei
  //  Bji = Ej*Ei_inv

  cv::Mat invEI;
  cv::invert(eI, invEI);
  *B = eJ * invEI;
  // std::cout<<*B<<std::endl;
  // std::cout<<"&&&&&&&&&&&&&&&&&&&"<<std::endl;

  // 释放内存
  picI.release();
  picJ.release();
  cmI.release();
  cmJ.release();
  eI.release();
  invEI.release();
  eJ.release();

  return true;
}

bool GetEMat(Mat pic, Mat cameraMatrix, Mat distCoffs, Size patternSize,
             Mat externalMat)
{
  cv::Size psz(patternSize.width, patternSize.height);
  return GetExternalMat(*pic, *cameraMatrix, *distCoffs, psz, *externalMat);
}

bool ProjectPoints(Mat objectPoints, Mat externalMat, Mat cameraMatrix,
                   Mat distCoeffs, Mat imagePoints)
{
  if (objectPoints->empty() || externalMat->empty() || cameraMatrix->empty() ||
      distCoeffs->empty())
  {
    return false;
  }
  if (externalMat->size[0] != 4 || externalMat->size[1] != 4)
  {
    return false;
  }

  cv::Mat rvec, tvec;
  tvec = (*externalMat)(cv::Rect(3, 0, 1, 3));
  cv::Mat rotateMat = (*externalMat)(cv::Rect(0, 0, 3, 3));
  // 旋转矩阵求rvec
  cv::Rodrigues(rotateMat, rvec);

  // 映射坐标
  cv::projectPoints(*objectPoints, rvec, tvec, *cameraMatrix, *distCoeffs,
                    *imagePoints);
  *imagePoints = (*imagePoints).reshape(2, (*objectPoints).size[0]);

  // 释放内存
  rvec.release();
  tvec.release();
  rotateMat.release();
  return true;
}

/**
 * return error code
 * -1 input mats empty
 * -2 found chessboard failed
 **/
int GetStereoBMat(Mats pics, Mats cameraMatrixs, Mats distCoffs, Size patternSize,
                  Mat B)
{
  if (pics.length != 2 || cameraMatrixs.length != 2 || distCoffs.length != 2)
  {
    return -1;
  }
  cv::Mat imgs[2]{*(pics.mats[0]), *(pics.mats[1])};
  if (imgs[0].empty() || imgs[1].empty())
  {
    return -1;
  }
  cv::Mat cmI = *(cameraMatrixs.mats[0]);
  cv::Mat cmJ = *(cameraMatrixs.mats[1]);
  if (cmI.empty() || cmJ.empty())
  {
    return -1;
  }

  cv::Mat distCoeffs[2]{*(distCoffs.mats[0]), *(distCoffs.mats[1])};
  if (distCoeffs[0].empty() || distCoeffs[1].empty())
  {
    return -1;
  }

  cv::Size boardSize(patternSize.width, patternSize.height);
  cv::Size imageSize = imgs[0].size();

  const int maxScale = 2;

  std::vector<std::vector<cv::Point2f>> imagePoints[2];
  imagePoints[0].resize(1);
  imagePoints[1].resize(1);

  int countFound = 0;
  for (int i = 0; i < 2; ++i)
  {
    bool found = false;
    std::vector<cv::Point2f> &corners = imagePoints[i][0];
    for (int scale = 1; scale <= maxScale; scale++)
    {
      cv::Mat timg;
      if (scale == 1)
        timg = imgs[i];
      else
        cv::resize(imgs[i], timg, cv::Size(), scale, scale, cv::INTER_LINEAR_EXACT);
      found = findChessboardCorners(timg, boardSize, corners,
                                    cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_ACCURACY);
      if (found)
      {
        if (scale > 1)
        {
          cv::Mat cornersMat(corners);
          cornersMat *= 1. / scale;
        }
        break;
      }
    }

    if (!found)
    {
      break;
    }
    else
    {
      countFound += 1;
    }

    cornerSubPix(imgs[i], corners, cv::Size(11, 11), cv::Size(-1, -1),
                 cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
                                  30, 0.01));
  }

  if (countFound < 2)
  {
    std::cout << "get corners failed" << std::endl;
    return -1;
  }

  // 初始化棋盘格角点的世界坐标
  std::vector<std::vector<cv::Point3f>> objectCorners(1);
  int h = patternSize.height, w = patternSize.width;
  for (int i = 0; i < h; i++)
  {
    for (int j = 0; j < w; j++)
    {
      objectCorners[0].emplace_back(cv::Point3f(i, j, 0));
    }
  }

  cv::Mat R, T, E, F;
  std::cout << "steroe calibrating..." << std::endl;
  //std::cout<<imagePoints[0]<<std::endl<<std::endl;
  std::cout << imagePoints[1].size() << std::endl;
  std::cout << cmI << std::endl
            << cmJ << std::endl;
  double rms = stereoCalibrate(objectCorners, imagePoints[0], imagePoints[1],
                               cmI, distCoeffs[0],
                               cmJ, distCoeffs[1],
                               imageSize, R, T, E, F,
                               cv::CALIB_FIX_ASPECT_RATIO +
                                   cv::CALIB_ZERO_TANGENT_DIST +
                                   cv::CALIB_USE_INTRINSIC_GUESS +
                                   cv::CALIB_SAME_FOCAL_LENGTH +
                                   cv::CALIB_RATIONAL_MODEL +
                                   cv::CALIB_FIX_K3 + cv::CALIB_FIX_K4 + cv::CALIB_FIX_K5,
                               cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, 1e-5));
  std::cout << "done with RMS error=" << rms << std::endl;
  std::cout << R << std::endl;
  std::cout << T << std::endl;
  *B = cv::Mat::zeros(4, 4, CV_64FC1);
  R.copyTo((*B)(cv::Rect(0, 0, 3, 3)));
  T.copyTo((*B)(cv::Rect(3, 0, 1, 3)));
  (*B).at<double>(3, 3) = 1;

  // CALIBRATION QUALITY CHECK
  // because the output fundamental matrix implicitly
  // includes all the output information,
  // we can check the quality of calibration using the
  // epipolar geometry constraint: m2^t*F*m1=0
  //      double err = 0;
  //      int npoints = 0;
  //      std::vector<cv::Vec3f> lines[2];
  //      for( int i = 0; i < 1; i++ )
  //      {
  //          int npt = (int)imagePoints[0][i].size();
  //          Mat imgpt[2];
  //          for( int k = 0; k < 2; k++ )
  //          {
  //              imgpt[k] = Mat(imagePoints[k][i]);
  //              cv::undistortPoints(imgpt[k], imgpt[k], cameraMatrix[k], distCoeffs[k], Mat(), cameraMatrix[k]);
  //              cv::computeCorrespondEpilines(imgpt[k], k+1, F, lines[k]);
  //          }
  //          for( int j = 0; j < npt; j++ )
  //          {
  //              double errij = fabs(imagePoints[0][i][j].x*lines[1][j][0] +
  //                                  imagePoints[0][i][j].y*lines[1][j][1] + lines[1][j][2]) +
  //                             fabs(imagePoints[1][i][j].x*lines[0][j][0] +
  //                                  imagePoints[1][i][j].y*lines[0][j][1] + lines[0][j][2]);
  //              err += errij;
  //          }
  //          npoints += npt;
  //      }
  //      std::cout << "average epipolar err = " <<  err/npoints << std::endl;
  return 0;
}