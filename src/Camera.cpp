//
// Created by rain on 17-12-19.
//

#include "Camera.h"

namespace PL_VO
{

Camera::Camera(const string &strSettingsFile)
{
    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);

    if (!fsSettings.isOpened())
    {
        cerr << "Failed to open settings file at " << strSettingsFile << endl;
        exit(-1);
    }

    mfx = fsSettings["Camera.fx"];
    mfy = fsSettings["Camera.fy"];
    mcx = fsSettings["Camera.cx"];
    mcy = fsSettings["Camera.cy"];

    mk1 = fsSettings["Camera.k1"];
    mk2 = fsSettings["Camera.k2"];
    mp1 = fsSettings["Camera.p1"];
    mp2 = fsSettings["Camera.p2"];
    mk3 = fsSettings["Camera.k3"];

    LOG_IF(WARNING, mk3 ==0) << "the camera calibration parameter(k3) is zero";

    mImageHeight = fsSettings["Camera.height"];
    mImageWidth = fsSettings["Camera.width"];

    mImageGridHeight = fsSettings["ImageGridHeight"];
    mImageGridWidth = fsSettings["ImageGridWidth"];

    mnumFeatures = fsSettings["ORBextractor.numFeatures"];
    minDist = fsSettings["ORBextractor.minDist"];
    mFeatureShow = fsSettings["Viewer.FeatureShow"];

    mdepthscale = fsSettings["Camera.depth_scale"];
}

Camera::Camera()
{
}

/**
* @brief dist
* @param p the point of the normalized coordinate
* @param pCorrected the point have been corrected
*/
void Camera::Distortion(const Eigen::Vector2d & p, Eigen::Vector2d & du)
{
    double x2, y2, xy, r2, rad_dist;

    x2 = p[0] * p[0];
    y2 = p[1] * p[1];
    xy = p[0] * p[1];
    r2 = x2 + y2;

    // NOTICE: there is no 1, because this calculation is the corrected quantity, not the corrected point.
    rad_dist = mk1*r2 + mk2*r2*r2;

    du[0] = p[0]*rad_dist + 2.0*mp1*xy + mp2*(r2 + 2.0*x2);
    du[1] = p[1]*rad_dist + 2.0*mp2*xy + mp1*(r2 + 2.0*y2);
}

void Camera::LiftProjective(const Eigen::Vector2d &p, Eigen::Vector3d &P)
{
    double invfx, invfy, invK13, invK23;
    double xd, yd;
    double xu, yu;

    invfx = 1.0 / mfx;
    invfy = 1.0 / mfy;
    invK13 = mcx / mfx;
    invK23 = mcy / mfy;

    xd = invfx*p(0) - invK13;
    yd = invfy*p(1) - invK23;

    if (1)
    {
        // Recursive distortion model
        int n = 8;
        Eigen::Vector2d du;

        Distortion(Eigen::Vector2d(xd, yd), du);

        xu = xd - du(0);
        yu = yd - du(1);

        // the
        for (int i = 0; i < n; i++)
        {
            Distortion(Eigen::Vector2d(xd, yd), du);
            xu = xd - du(0);
            yu = yd - du(1);
        }
    }
    else
    {
        xu = xd;
        yu = yd;
    }

    P << xu, yu, 1.0;
}

void Camera::InitUndistortRectifyMap(cv::Mat &undistmap1, cv::Mat &undistmap2)
{
    cv::Mat K;
    cv::Size imageSize;
    cv::Mat DistCoef(5,1,CV_32F);

    imageSize.height = mImageHeight;
    imageSize.width = mImageWidth;

    K = Converter::toCvMat(GetCameraIntrinsic());

    // there must be five pareameters, otherwise the undistort image is abnormal.
    DistCoef.at<float>(0) = (float)GetDistortionPara2()[0];
    DistCoef.at<float>(1) = (float)GetDistortionPara2()[1];
    DistCoef.at<float>(2) = (float)GetDistortionPara2()[2];
    DistCoef.at<float>(3) = (float)GetDistortionPara2()[3];
    DistCoef.at<float>(4) = (float)GetDistortionPara2()[4];

    cv::initUndistortRectifyMap(K, DistCoef, cv::Mat(),
                                cv::getOptimalNewCameraMatrix(K, DistCoef, imageSize, 1, imageSize, 0),
                                imageSize, CV_16SC2, undistmap1, undistmap2);
}

} // namespace PL_VO
