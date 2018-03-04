//
// Created by rain on 17-12-29.
//

#include "Tracking.h"

namespace PL_VO
{

Tracking::Tracking(Camera *pCamera) : mpcamera(pCamera)
{
    mpLineFeature = new(LineFeature);
    mpPointFeature = new(PointFeature);

    countMapPoint = 0;
    countMapLine = 0;

    mPoseInc.setQuaternion(Eigen::Quaterniond::Identity());
    mPoseInc.translation().setZero();;
}

Tracking::~Tracking()
{
    delete(mpLineFeature);
    delete(mpPointFeature);
}

void Tracking::SetMap(Map *pMap)
{
    mpMap = pMap;
}

// just use for the test.
void Tracking::SetCurLastFrame(Frame *pcurFrame, Frame *plastFrame)
{
    mpcurrentFrame = pcurFrame;
    mplastFrame = plastFrame;
}

// just use for the test
void Tracking::SetLastKeyFrame(KeyFrame *pKeyFrame)
{
    mpLastKeyFrame = pKeyFrame;
}

void Tracking::Track(const cv::Mat &imagergb, const cv::Mat &imD, const double &timeStamps)
{
    mimageGray = imagergb.clone();
    mimagergb = imagergb.clone();
    mimageDepth = imD;

    bool mbRGB = Config::imageRGBForm();
    if(mimageGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mimageGray,mimageGray,CV_RGB2GRAY);
        else
            cvtColor(mimageGray,mimageGray,CV_BGR2GRAY);
    }
    else if(mimageGray.channels()==4)
    {
        if(mbRGB)
            cvtColor(mimageGray,mimageGray,CV_RGBA2GRAY);
        else
            cvtColor(mimageGray,mimageGray,CV_BGRA2GRAY);
    }

    mpcurrentFrame = new Frame(timeStamps, mpcamera, mpLineFeature, mpPointFeature);

    if (mpcurrentFrame->GetFrameID() == 0)
    {
        mpcurrentFrame->Tcw.so3().setQuaternion(Eigen::Quaterniond::Identity());
        mpcurrentFrame->Tcw.translation() = Eigen::Vector3d(0, 0, 0);
    }

    mpcurrentFrame->detectFeature(mimageGray, mimageDepth);

    mpcurrentFrame->UndistortKeyFeature();

    mpcurrentFrame->UnprojectStereo(mimageDepth);

    if (!mlastimageGrays.empty())
    {
        vector<cv::DMatch> vpointMatches;
        vector<cv::DMatch> vpointRefineMatches;
        vector<cv::DMatch> vlineMatches;
        vector<cv::DMatch> vlineRefineMatches;

        mpcurrentFrame->matchLPFeature(mpLastKeyFrame->mpointDesc, mpcurrentFrame->mpointDesc, vpointMatches,
                                       mpLastKeyFrame->mlineDesc, mpcurrentFrame->mlineDesc, vlineMatches);

        mpcurrentFrame->refineLPMatches(mpLastKeyFrame->mvKeyPoint, mpcurrentFrame->mvKeyPoint,
                                        mpLastKeyFrame->mvKeyLine, mpcurrentFrame->mvKeyLine,
                                        vpointMatches, vpointRefineMatches, vlineMatches, vlineRefineMatches);

//        mpLastKeyFrame->UnprojectStereo(mlastimageDepth, vpointRefineMatches, vlineRefineMatches);

        // use the pnp and point match to track the reference frame
        // use the pnp ransanc to remove the outliers
        TrackRefFrame(vpointRefineMatches, vlineRefineMatches);
        // TrackRefFrameByPnP(vpointRefineMatches, vlineRefineMatches);

        UpdateMapLPfeature(vpointRefineMatches, vlineRefineMatches);

//        mpcurrentFrame->MapLinePointShow();

        Optimizer::PoseOptimization(mpcurrentFrame, mpLastKeyFrame);

        cv::Mat showimg;
        cv::Mat showimg2;
        cv::drawMatches(mlastimagergb, mpLastKeyFrame->mvKeyPoint, mimagergb, mpcurrentFrame->mvKeyPoint,
                        vpointRefineMatches, showimg2);

        std::vector<char> mask(vlineRefineMatches.size(), 1);
        cv::line_descriptor::drawLineMatches(mlastimagergb, mpLastKeyFrame->mvKeyLine, mimagergb, mpcurrentFrame->mvKeyLine,
                                             vlineRefineMatches, showimg,  cv::Scalar::all(-1), cv::Scalar::all(-1), mask,
                                             cv::line_descriptor::DrawLinesMatchesFlags::DEFAULT);
        cv::imshow("line match", showimg);
        cv::imshow("point match", showimg2);
        cv::waitKey(5);
    }

    if (NeedNewKeyFrame())
    {
        CreateNewKeyFrame();
        mlastimageGrays = mimageGray.clone();
        mlastimagergb = mimagergb.clone();
        mlastimageDepth = mimageDepth.clone();
    }

    mpMap->mvpFrames.push_back(mpcurrentFrame);
    mplastFrame = new Frame(*mpcurrentFrame);

    if (!mimagergb.empty())
        mpViewer->UpdateShowImage(this);

    if (mpMapDrawer)
        mpMapDrawer->SetCurrentCameraPose(mpcurrentFrame->Tcw.inverse().matrix3x4());

    if (mpViewer)
        mpViewer->Release();

}

bool Tracking::TrackRefFrame(const vector<cv::DMatch> &vpointMatches, const vector<cv::DMatch> &vlineMatches)
{
    vector<PointFeature2D *> vpPointFeature2DLast;
    vector<PointFeature2D *> vpPointFeature2DCur;
    vector<LineFeature2D *> vpLineFeature2DLast;
    vector<LineFeature2D *> vpLineFeature2DCur;


    for (auto match : vpointMatches)
    {
        PointFeature2D *pPointFeature2DLast = mpLastKeyFrame->mvpPointFeature2D[match.queryIdx];
        PointFeature2D *pPointFeature2DCur = mpcurrentFrame->mvpPointFeature2D[match.trainIdx];

        CHECK_NOTNULL(pPointFeature2DLast);
        CHECK_NOTNULL(pPointFeature2DCur);

        if (!pPointFeature2DLast->mbinlier)
        {
            // TODO if point feature is not inlier in the last frame, the point feature is set outlier in the current frame
//            pPointFeature2DCur->mbinlier = false;
            continue;
        }

        vpPointFeature2DLast.emplace_back(pPointFeature2DLast);
        vpPointFeature2DCur.emplace_back(pPointFeature2DCur);
    }

    for (auto match : vlineMatches)
    {
        LineFeature2D *pLineFeature2DLast = mpLastKeyFrame->mvpLineFeature2D[match.queryIdx];
        LineFeature2D *pLineFeature2DCur = mpcurrentFrame->mvpLineFeature2D[match.trainIdx];

        CHECK_NOTNULL(pLineFeature2DLast);
        CHECK_NOTNULL(pLineFeature2DCur);

        // TODO if line feature is not inlier in the last frame, the line feature is set outlier in the current frame, && !pLineFeature2DCur->mbinlier
        if (!pLineFeature2DLast->mbinlier)
        {
//            pLineFeature2DCur->mbinlier = false;
            continue;
        }


        vpLineFeature2DLast.emplace_back(pLineFeature2DLast);
        vpLineFeature2DCur.emplace_back(pLineFeature2DCur);
    }

//    cout << "PoseInc: " << endl << mPoseInc << endl;

    Optimizer::PnPResultOptimization(mpcurrentFrame, mPoseInc, vpPointFeature2DLast, vpPointFeature2DCur,
                                     vpLineFeature2DLast, vpLineFeature2DCur);

//    cout << "Optimization PoseInc: " << endl << mPoseInc << endl;

    mpcurrentFrame->SetPose(mpLastKeyFrame->GetPose()*mPoseInc);

}

bool Tracking::TrackRefFrameByPnP(const vector<cv::DMatch> &vpointMatches, const vector<cv::DMatch> &vlineMatches)
{
    vector<cv::Point3d> vPoint3d;
    vector<cv::Point2d> vPoint2d;
    vector<PointFeature2D *> vpPointFeature2DLast;
    vector<PointFeature2D *> vpPointFeature2DCur;
    vector<LineFeature2D *> vpLineFeature2DLast;
    vector<LineFeature2D *> vpLineFeature2DCur;

    for (auto match : vpointMatches)
    {
        PointFeature2D *pPointFeature2DLast = mpLastKeyFrame->mvpPointFeature2D[match.queryIdx];
        PointFeature2D *pPointFeature2DCur = mpcurrentFrame->mvpPointFeature2D[match.trainIdx];

        if (!pPointFeature2DLast->mbinlier || !pPointFeature2DCur->mbinlier)
        {
            continue;
        }

        vPoint3d.emplace_back(Converter::toCvPoint3f(pPointFeature2DLast->mPoint3dw));
        vPoint2d.emplace_back(Converter::toCvPoint2f(pPointFeature2DCur->mpixel));
        vpPointFeature2DLast.emplace_back(pPointFeature2DLast);
        vpPointFeature2DCur.emplace_back(pPointFeature2DCur);
    }

    for (auto match : vlineMatches)
    {
        LineFeature2D *pLineFeature2DLast = mpLastKeyFrame->mvpLineFeature2D[match.queryIdx];
        LineFeature2D *pLineFeature2DCur = mpcurrentFrame->mvpLineFeature2D[match.trainIdx];

        CHECK_NOTNULL(pLineFeature2DLast);
        CHECK_NOTNULL(pLineFeature2DCur);

        // TODO if line feature is not inlier in the last frame, the line feature is set outlier in the current frame, && !pLineFeature2DCur->mbinlier
        if (!pLineFeature2DLast->mbinlier)
        {
//            pLineFeature2DCur->mbinlier = false;
            continue;
        }


        vpLineFeature2DLast.emplace_back(pLineFeature2DLast);
        vpLineFeature2DCur.emplace_back(pLineFeature2DCur);
    }

    cv::Mat rvec, tvec;
    vector<int> vinliers;
    vector<int> voutliers;
    cv::solvePnPRansac(vPoint3d, vPoint2d, PL_VO::Converter::toCvMat(mpcamera->GetCameraIntrinsic()), cv::Mat(),
                       rvec, tvec, false, 100, 4.0, 0.99, vinliers);

    voutliers.resize(vpPointFeature2DCur.size());

    for (int i = 0; i < voutliers.size(); i++)
    {
        voutliers.at(i) = -1;
    }

    for (auto inlier : vinliers)
    {
        voutliers.at(inlier) = 1;
    }

    for (int i = 0; i < voutliers.size(); i++)
    {
        if (voutliers.at(i) == -1)
        {
            vpPointFeature2DCur[i]->mbinlier = false;
//            vpPointFeature2DLast[i]->mbinlier = false;
        }
    }

    mpcurrentFrame->Tcw = Sophus::SE3d(Sophus::SO3d::exp(Eigen::Vector3d(rvec.at<double>(0,0), rvec.at<double>(1,0), rvec.at<double>(2,0))),
                                       Converter::toVector3d(tvec));

    mPoseInc = mpLastKeyFrame->GetPose().inverse()*mpcurrentFrame->GetPose();

    Optimizer::PnPResultOptimization(mpcurrentFrame, mPoseInc, vpPointFeature2DLast, vpPointFeature2DCur,
                                     vpLineFeature2DLast, vpLineFeature2DCur);

    mpcurrentFrame->SetPose(mpLastKeyFrame->GetPose()*mPoseInc);


}

void Tracking::UpdateMapPointfeature(const vector<cv::DMatch> &vpointMatches)
{
    for (auto match : vpointMatches)
    {
        PointFeature2D *pPointFeature = mpLastKeyFrame->mvpPointFeature2D[match.queryIdx];

        CHECK_NOTNULL(pPointFeature);

        if (pPointFeature->mpMapPoint == nullptr)
        {
            MapPoint *pMapPoint = new MapPoint;

            pMapPoint->mID = countMapPoint;
            if (!pPointFeature->mPoint3dw.isZero())
                pMapPoint->mPosew = mpLastKeyFrame->Tcw.inverse()*pPointFeature->mPoint3dw;
            pMapPoint->mmpPointFeature2D[mpLastKeyFrame->GetFrameID()] = pPointFeature;
            pMapPoint->mvpFrameinvert.push_back(mpLastKeyFrame->mpFrame);

            mpLastKeyFrame->mpFrame->mvpMapPoint.push_back(pMapPoint);
            pPointFeature->mpMapPoint = pMapPoint;

            countMapPoint++;

            PointFeature2D *pcurPointFeature = mpcurrentFrame->mvpPointFeature2D[match.trainIdx];
            CHECK_NOTNULL(pcurPointFeature);

            pMapPoint->mmpPointFeature2D[mpcurrentFrame->GetFrameID()] = pcurPointFeature;
            pMapPoint->mvpFrameinvert.push_back(mpcurrentFrame);

            mpcurrentFrame->mvpMapPoint.push_back(pMapPoint);
            pcurPointFeature->mpMapPoint = pMapPoint;

        }
        else // if (pPointFeature->mpMapPoint == nullptr)
        {
            MapPoint *pMapPoint = pPointFeature->mpMapPoint;

            PointFeature2D *pcurPointFeature = mpcurrentFrame->mvpPointFeature2D[match.trainIdx];
            CHECK_NOTNULL(pcurPointFeature);

            auto it = pMapPoint->mmpPointFeature2D.find(mpcurrentFrame->GetFrameID());
            if (it != pMapPoint->mmpPointFeature2D.end())
            {
                LOG(ERROR) << "the point feature2d exist " << endl;
            }

            pMapPoint->mmpPointFeature2D[mpcurrentFrame->GetFrameID()] = pcurPointFeature;
            pMapPoint->mvpFrameinvert.push_back(mpcurrentFrame);

            if (pMapPoint->mPosew.isZero())
            {
//                if (!pcurPointFeature->mPoint3dw.isZero())
//                {
//                    pMapPoint->mPosew = mpcurrentFrame->Tcw.inverse()*pcurPointFeature->mPoint3dw;
//                    cout << "use the current frame's observation to updae the MapPoint: " << pcurPointFeature->mPoint3dw.transpose() << endl;
//                }
            }

            mpcurrentFrame->mvpMapPoint.push_back(pMapPoint);
            pcurPointFeature->mpMapPoint = pMapPoint;

            if (pMapPoint->GetObservedNum() > 2)
            {
                mpMap->AddMapPoint(pMapPoint);
            }

        } // if (pPointFeature->mpMapPoint == nullptr)

    } // for (auto match : vpointMatches)

}

void Tracking::UpdateMapLinefeature(const vector<cv::DMatch> &vlineMatches)
{

    for (auto match : vlineMatches)
    {
        LineFeature2D *pLineFeature = mpLastKeyFrame->mvpLineFeature2D[match.queryIdx];

        CHECK_NOTNULL(pLineFeature);

        if (pLineFeature->mpMapLine == nullptr)
        {
            MapLine *pMapLine = new MapLine;
            pMapLine->mID = countMapLine;

            if (!pLineFeature->mStartPoint3dw.isZero())
                pMapLine->mPoseStartw = mpLastKeyFrame->Tcw.inverse()*pLineFeature->mStartPoint3dw;
            if (!pLineFeature->mEndPoint3dw.isZero())
                pMapLine->mPoseEndw = mpLastKeyFrame->Tcw.inverse()*pLineFeature->mEndPoint3dw;

            pMapLine->mvpFrameinvert.push_back(mpLastKeyFrame->mpFrame);
            pMapLine->mmpLineFeature2D[mpLastKeyFrame->GetFrameID()] = pLineFeature;

            mpLastKeyFrame->mpFrame->mvpMapLine.push_back(pMapLine);
            pLineFeature->mpMapLine = pMapLine;

            countMapLine++;

            LineFeature2D *pcurLineFeature = mpcurrentFrame->mvpLineFeature2D[match.trainIdx];
            CHECK_NOTNULL(pcurLineFeature);

            pMapLine->mmpLineFeature2D[mpcurrentFrame->GetFrameID()] = pcurLineFeature;
            pMapLine->mvpFrameinvert.push_back(mpcurrentFrame);

            mpcurrentFrame->mvpMapLine.push_back(pMapLine);
            pcurLineFeature->mpMapLine = pMapLine;

        }
        else // if (pLineFeature->pMapLine == nullptr)
        {
            MapLine *pMapLine = pLineFeature->mpMapLine;

            LineFeature2D *pcurLineFeature = mpcurrentFrame->mvpLineFeature2D[match.trainIdx];
            CHECK_NOTNULL(pcurLineFeature);

            auto it = pMapLine->mmpLineFeature2D.find(mpcurrentFrame->GetFrameID());
            if (it != pMapLine->mmpLineFeature2D.end())
            {
                LOG(ERROR) << "the point feature2d doesn't exist " << endl;
            }

            pMapLine->mmpLineFeature2D[mpcurrentFrame->GetFrameID()] = pcurLineFeature;
            pMapLine->mvpFrameinvert.push_back(mpcurrentFrame);

            if (pMapLine->mPoseStartw.isZero() || pMapLine->mPoseEndw.isZero())
            {
//                if (!pcurLineFeature->mStartPoint3dw.isZero())
//                {
//                    pMapLine->mPoseStartw =  mpcurrentFrame->Tcw.inverse()*pcurLineFeature->mStartPoint3dw;
//                    cout << "use the current frame's observation to updae the MapLine: " << pcurLineFeature->mStartPoint3dw.transpose() << endl;
//                }
//
//                if (!pcurLineFeature->mEndPoint3dw.isZero())
//                {
//                    pMapLine->mPoseEndw = mpcurrentFrame->Tcw.inverse()*pcurLineFeature->mEndPoint3dw;
//                    cout << "use the current frame's observation to updae the MapLine: " << pcurLineFeature->mEndPoint3dw.transpose() << endl;
//                }
            }

            mpcurrentFrame->mvpMapLine.push_back(pMapLine);
            pcurLineFeature->mpMapLine = pMapLine;

            if (!pMapLine->GetPoseEnd().isZero() && !pMapLine->GetPoseStart().isZero() && pMapLine->GetObservedNum() > 2)
            {
                mpMap->AddMapLine(pMapLine);
            }

        } // if (pLineFeature->pMapLine == nullptr)
    } // for (auto match : vlineMatches)
}

void Tracking::UpdateMapLPfeature(const vector<cv::DMatch> &vpointMatches, const vector<cv::DMatch> &vlineMatches)
{
    if (Config::plInParallel())
    {
        auto updateMapLine = async(launch::async, &Tracking::UpdateMapLinefeature, this, vlineMatches);
        auto updateMapPoint = async(launch::async, &Tracking::UpdateMapPointfeature, this, vpointMatches);
    }
    else
    {
        UpdateMapLinefeature(vlineMatches);
        UpdateMapPointfeature(vpointMatches);
    }
}

void Tracking::SetLocalMapping(LocalMapping *pLocalMapping)
{
    mpLocalMapping = pLocalMapping;
}

bool Tracking::NeedNewKeyFrame()
{
    if (mpcurrentFrame->GetFrameID() == 0)
        return true;

    int inlierscnt = 0;

    for (auto pPointFeature2D : mpcurrentFrame->mvpPointFeature2D)
    {
        if (pPointFeature2D == nullptr)
            continue;

        if (pPointFeature2D->mbinlier)
            inlierscnt++;
    }

    mpcurrentFrame->mpointinliersnum = inlierscnt;

    inlierscnt = 0;
    for (auto pLineFeature2D : mpcurrentFrame->mvpLineFeature2D)
    {
        if (pLineFeature2D == nullptr)
            continue;

        if (pLineFeature2D->mbinlier)
            inlierscnt++;
    }

    mpcurrentFrame->mlineinliersnum = inlierscnt;

    mpcurrentFrame->minliersnum = mpcurrentFrame->mpointinliersnum + mpcurrentFrame->mlineinliersnum;

    // TODO when test in the no texture datasets, the threshold should be reduced
    if (mpcurrentFrame->minliersnum < 200)
        return false;

    double r = mPoseInc.so3().log().norm();
    double t = mPoseInc.translation().norm();
    double T = r + 0.65*t;

    if (T < 0.03)
        return false;

    mpcurrentFrame->SetKeyFrameFlag(true);

    return true;
}

void Tracking::CreateNewKeyFrame()
{
    KeyFrame *pKeyFrame = new KeyFrame(*mpcurrentFrame, mpMap);

    pKeyFrame->mpFrame = mpcurrentFrame;

    mpcurrentFrame->mpKeyFrame = pKeyFrame;

    mpLastKeyFrame = pKeyFrame;

    mpLocalMapping->InsertKeyFrame(pKeyFrame);
}

cv::Mat Tracking::GetImageShow()
{
    return mimagergb;
}

void Tracking::SetViewer(Viewer *pViewer)
{
    mpViewer = pViewer;
}

void Tracking::SetMapDrawer(MapDrawer *pMapDrawer)
{
    mpMapDrawer = pMapDrawer;
}



} // namespace PL_VO
