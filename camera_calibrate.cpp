#include "camera_calibrate.h"

double GetInternalMat(Mats pics, Size patternSize, Mat cameraMatrix,
                      Mat distCoffs, bool accurcy)
{
  if (pics.length < 13)
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
                                   *cameraMatrix, *distCoffs, rvecs, tvecs);

  // 释放内存
  std::vector<cv::Mat>().swap(rvecs);
  std::vector<cv::Mat>().swap(tvecs);
  return res;
}

bool GetExternalMat(cv::Mat pic, cv::Mat cameraMatrix, cv::Mat distCoffs,
                    cv::Size patternSize, cv::Mat &external)
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
            cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER,
                             40, 0.001));

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

  cv::Mat invEI;
  cv::invert(eI, invEI);
  *B = eJ * invEI;

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

bool GetEMat(Mat pic, Mat cameraMatrix, Mat distCoffs, Size patternSize, Mat externalMat)
{
  cv::Size psz(patternSize.width, patternSize.height);
  return GetExternalMat(*pic, *cameraMatrix, *distCoffs, psz, *externalMat);
}

bool ProjectPoints(Mat objectPoints, Mat externalMat, Mat cameraMatrix, Mat distCoeffs, Mat imagePoints)
{
  if (objectPoints->empty() || externalMat->empty() || cameraMatrix->empty() || distCoeffs->empty())
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
  cv::projectPoints(*objectPoints, rvec, tvec, *cameraMatrix, *distCoeffs, *imagePoints);
  *imagePoints = (*imagePoints).reshape(2, (*objectPoints).size[0]);

  // 释放内存
  rvec.release();
  tvec.release();
  rotateMat.release();
  return true;
}
