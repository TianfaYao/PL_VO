//
// Created by rain on 18-1-25.
//

#include "KeyFrame.h"

namespace PL_VO
{

KeyFrame::KeyFrame(Frame &frame, Map *pMap)
{
    mpCamera = frame.mpCamera;
    mpMap = pMap;

    mID = frame.GetFrameID();
    mtimeStamp = frame.mtimeStamp;

    mImageHeight = mpCamera->mImageHeight;
    mImageWidth = mpCamera->mImageWidth;

    Tcw = frame.Tcw;
    Twc = frame.Twc;

    mpointDesc = frame.mpointDesc.clone();
    mlineDesc = frame.mlineDesc.clone();

    mvKeyPoint.assign(frame.mvKeyPoint.begin(), frame.mvKeyPoint.end());
    mvKeyLine.assign(frame.mvKeyLine.begin(), frame.mvKeyLine.end());
    mvKeyPointUn.assign(frame.mvKeyPointUn.begin(), frame.mvKeyPointUn.end());
    mvKeyLineUn.assign(frame.mvKeyLineUn.begin(), frame.mvKeyLineUn.end());
    mvpMapPoint.assign(frame.mvpMapPoint.begin(), frame.mvpMapPoint.end());
    mvpMapLine.assign(frame.mvpMapLine.begin(), frame.mvpMapLine.end());
    mvpPointFeature2D.assign(frame.mvpPointFeature2D.begin(), frame.mvpPointFeature2D.end());
    mvpLineFeature2D.assign(frame.mvpLineFeature2D.begin(), frame.mvpLineFeature2D.end());

    mbFirstConnection = true;
    mbBad = false;
}

size_t KeyFrame::GetFrameID()
{
    unique_lock<mutex> lock(mMutexPose);
    return mID;
}

double KeyFrame::FindDepth(const cv::Point2f &point, const cv::Mat &imagedepth)
{
    int x = cvRound(point.x);
    int y = cvRound(point.y);

    CHECK(x >= 0 && x <= mImageWidth);
    CHECK(y >= 0 && y <= mImageHeight);

    ushort d = imagedepth.ptr<ushort>(y)[x];

    if (d!=0)
    {
        return double(d)/mpCamera->mdepthscale;
    }
    else
    {
        int dx[4] = {-1,0,1,0};
        int dy[4] = {0,-1,0,1};
        for (int i = 0; i < 4; i++)
        {
            if ((x+dx[i]) < 0 || (x+dx[i]) > mImageWidth)
            {
                continue;
            }
            if ((y+dy[i]) < 0 || (y+dy[i]) > mImageHeight)
            {
                continue;
            }

            d = imagedepth.ptr<ushort>(y+dy[i])[x+dx[i]];
            if (d != 0)
            {
                return double(d)/mpCamera->mdepthscale;
            }
        }
    }

    return -1.;
}

void KeyFrame::UnprojectPointStereo(const cv::Mat &imageDepth, const vector<cv::DMatch> &vpointMatches)
{
    mvpPointFeature2D.resize(mvKeyPointUn.size());

    for (auto match : vpointMatches)
    {

        int idxMatch;
        cv::KeyPoint kp;

        idxMatch = match.queryIdx;  // the last frame

        if (mvpPointFeature2D[idxMatch] == nullptr)
        {
            kp = mvKeyPointUn[idxMatch];

            // !!!notice: the pixel of the depth image should use the distorted image
            Eigen::Vector3d Point3dw;
            double d = FindDepth(mvKeyPoint[idxMatch].pt, imageDepth);

            PointFeature2D *ppointFeature = new PointFeature2D(Converter::toVector2d(kp.pt), kp.octave, kp.response, idxMatch);

            if (d > 0)
            {
                Point3dw = mpCamera->Pixwl2World(Converter::toVector2d(kp.pt), Eigen::Quaterniond::Identity(), Eigen::Vector3d(0, 0, 0), d);
            }
            else
            {
                Point3dw.setZero();
                ppointFeature->mbinlier = false;
            }

            ppointFeature->mPoint3dw = Point3dw;

            mvpPointFeature2D[idxMatch] = ppointFeature;
        }
    } // for (auto match : vpointMatches)
}

void KeyFrame::UnprojectLineStereo(const cv::Mat &imageDepth, const vector<cv::DMatch> &vlineMatches)
{
    mvpLineFeature2D.resize(mvKeyLineUn.size());

    for (auto match : vlineMatches)
    {
        int idxMatch;
        cv::line_descriptor::KeyLine klUn;
        cv::line_descriptor::KeyLine kl;

         idxMatch = match.queryIdx; // the last frame

        if (mvpLineFeature2D[idxMatch] == nullptr)
        {
            klUn = mvKeyLineUn[idxMatch];
            kl = mvKeyLine[idxMatch];

            cv::Point2f startPointUn2f;
            cv::Point2f endPointUn2f;
            cv::Point2f startPoint2f;
            cv::Point2f endPoint2f;

            startPointUn2f = cv::Point2f(klUn.startPointX, klUn.startPointY);
            endPointUn2f = cv::Point2f(klUn.endPointX, klUn.endPointY);

            startPoint2f = cv::Point2f(kl.startPointX, kl.startPointY);
            endPoint2f = cv::Point2f(kl.endPointX, kl.endPointY);

            // !!!notice: use the distored image
            double d1 = FindDepth(startPoint2f, imageDepth);
            double d2 = FindDepth(endPoint2f, imageDepth);
            Eigen::Vector3d startPoint3dw;
            Eigen::Vector3d endPoint3dw;

            LineFeature2D *plineFeature2D = new LineFeature2D(Converter::toVector2d(startPointUn2f), Converter::toVector2d(endPointUn2f),
                                                              kl.octave, kl.response, idxMatch);

            if (d1 > 0)
            {

                startPoint3dw = mpCamera->Pixwl2World(Converter::toVector2d(startPointUn2f), Eigen::Quaterniond::Identity(),
                                                      Eigen::Vector3d(0, 0, 0), d1);
            }
            else
            {
                startPoint3dw.setZero(); // to set the pose zero and use the other observation to calculate the pose
                plineFeature2D->mbinlier = false;
            }

            if (d2 > 0)
            {
                endPoint3dw = mpCamera->Pixwl2World(Converter::toVector2d(endPointUn2f), Eigen::Quaterniond::Identity(),
                                                    Eigen::Vector3d(0, 0, 0), d2);
            }
            else
            {
                endPoint3dw.setZero();
                plineFeature2D->mbinlier = false;
            }

            plineFeature2D->mStartPoint3dw = startPoint3dw;
            plineFeature2D->mEndPoint3dw = endPoint3dw;

            mvpLineFeature2D[idxMatch]= plineFeature2D;

        } // if (mvpLineFeature2D[idxMatch] == nullptr)
    }
}

void KeyFrame::UnprojectStereo(const cv::Mat &imageDepth, const vector<cv::DMatch> vpointMatches,
                               const vector<cv::DMatch> vlineMatches)
{
    if (Config::plInParallel())
    {
        auto pointStereo = async(launch::async, &KeyFrame::UnprojectPointStereo, this, imageDepth, vpointMatches);
        auto lineStereo = async(launch::async, &KeyFrame::UnprojectLineStereo, this, imageDepth, vlineMatches);

        pointStereo.wait();
        lineStereo.wait();
    }
    else
    {
        UnprojectPointStereo(imageDepth, vpointMatches);
        UnprojectLineStereo(imageDepth, vlineMatches);
    }
}

void KeyFrame::AddConnection(KeyFrame *pKeyFrame, const int &weight)
{
    {
        unique_lock<mutex> lock(mMutexConnections);

        if (!mConnectedKeyFrameWeights.count(pKeyFrame))
            mConnectedKeyFrameWeights[pKeyFrame] = weight;
        else if (mConnectedKeyFrameWeights[pKeyFrame] != weight)
            mConnectedKeyFrameWeights[pKeyFrame] = weight;
        else
            return;
    }

    UpdateBestCovisibles();
}

void KeyFrame::UpdateBestCovisibles()
{
    unique_lock<mutex> lock(mMutexConnections);

    vector<pair<int, KeyFrame*> > vPairs;
    vPairs.reserve(mConnectedKeyFrameWeights.size());

    for (auto mit : mConnectedKeyFrameWeights)
        vPairs.push_back(make_pair(mit.second, mit.first));

    sort(vPairs.begin(), vPairs.end());

    list<KeyFrame *> lKeyFrames;
    list<int> lWeights;

    for (size_t i = 0, iend = vPairs.size(); i < iend; i++)
    {
        lKeyFrames.push_front(vPairs[i].second);
        lWeights.push_front(vPairs[i].first);
    }

    mvpOrderedConnectedKeyFrames = vector<KeyFrame*>(lKeyFrames.begin(), lKeyFrames.end());
    mvOrderedWeights = vector<int>(lWeights.begin(), lWeights.end());
}

void KeyFrame::AddChild(KeyFrame *pKeyFrame)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mspChildrens.insert(pKeyFrame);
}

void KeyFrame::UpdateConnections()
{
    map<Frame *, int> KFcounter;

    vector<MapPoint *> vpMapPoint;
    vector<MapLine *> vpMapLine;

    {
        unique_lock<mutex> lockMapLP(mMutexFeatures);
        vpMapLine.assign(mvpMapLine.begin(), mvpMapLine.end());
        vpMapPoint.assign(mvpMapPoint.begin(), mvpMapPoint.end());
    }

    for (auto pMapPoint : vpMapPoint)
    {
        if (!pMapPoint)
            continue;

        if (pMapPoint->isBad())
            continue;

        vector<Frame *> vpObservedFrame = pMapPoint->GetObservedFrame();

        for (auto pObservedFrame : vpObservedFrame)
        {
            if (pObservedFrame->GetFrameID() == mID)
                continue;

            if (!pObservedFrame->isKeyFrame())
                continue;

            KFcounter[pObservedFrame]++;
        }

    }

    for (auto pMapLine : vpMapLine )
    {
        if (!pMapLine)
            continue;

        if (pMapLine->isBad())
            continue;

        vector<Frame *> vpObservedFrame = pMapLine->GetObservedFrame();

        for (auto pObservedFrame : vpObservedFrame)
        {
            if (pObservedFrame->GetFrameID() == mID)
                continue;

            if (!pObservedFrame->isKeyFrame())
                continue;

            KFcounter[pObservedFrame]++;
        }
    }

    if (KFcounter.empty())
    {
        LOG(ERROR) << "no MapPoint and LinePoint between the current KeyFrame with the other KeyFrame" << endl;
        return;
    }

    int nmax = 0;
    Frame *pFrameMax = nullptr;
    int th = 60;

    vector<pair<int, KeyFrame *> > vPairs;

    for (auto &mit : KFcounter)
    {
        if (mit.second > nmax)
        {
            nmax = mit.second;
            pFrameMax = mit.first;
        }

        if (mit.second >= th)
        {
            CHECK_NOTNULL(mit.first->mpKeyFrame);
            vPairs.push_back(make_pair(mit.second, mit.first->mpKeyFrame));
            mit.first->mpKeyFrame->AddConnection(this, mit.second);
        }
    }

    if (vPairs.empty())
    {
        LOG(WARNING) << "no keyframe counter is over threshold" << endl;
        CHECK_NOTNULL(pFrameMax->mpKeyFrame);

        vPairs.push_back(make_pair(nmax, pFrameMax->mpKeyFrame));
        pFrameMax->mpKeyFrame->AddConnection(this, nmax);
    }

    sort(vPairs.begin(), vPairs.end());

    list<KeyFrame *> lKeyFrames;
    list<int> lWeights;

    for (size_t i = 0, iend = vPairs.size(); i < iend; i++)
    {
        lKeyFrames.push_front(vPairs[i].second);
        lWeights.push_front(vPairs[i].first);
    }

    {
        unique_lock<mutex> lockCon(mMutexConnections);

        for (auto &mit : KFcounter)
        {
            mConnectedKeyFrameWeights[mit.first->mpKeyFrame] = mit.second;
        }
        mvpOrderedConnectedKeyFrames = vector<KeyFrame*>(lKeyFrames.begin(), lKeyFrames.end());
        mvOrderedWeights = vector<int>(lWeights.begin(), lWeights.end());

        if (mbFirstConnection && mID != 0)
        {
            mpParent = mvpOrderedConnectedKeyFrames.front();
            mpParent->AddChild(this);
            mbFirstConnection = false;
        }
    }
} // void KeyFrame::UpdateConnections()

vector<KeyFrame*> KeyFrame::GetVectorCovisibleKeyFrames()
{
    unique_lock<mutex> lock(mMutexConnections);
    return mvpOrderedConnectedKeyFrames;
}

vector<MapPoint*> KeyFrame::GetMapPointMatches()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mvpMapPoint;
}

vector<MapLine*> KeyFrame::GetMapLineMatches()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mvpMapLine;
}

Sophus::SE3d KeyFrame::GetPose()
{
    unique_lock<mutex> lock(mMutexPose);
    return Tcw;
}

Eigen::Vector3d KeyFrame::GetCameraCenter()
{
    unique_lock<mutex> lock(mMutexPose);
    return Tcw.inverse().translation();
}

void KeyFrame::SetPose(Sophus::SE3d Tcw_)
{
    unique_lock<mutex> lock(mMutexPose);
    Tcw = Tcw_;
}

bool KeyFrame::isBad()
{
    unique_lock<mutex> lock(mMutexConnections);
    return mbBad;
}

KeyFrame* KeyFrame::GetParent()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mpParent;
}

vector<KeyFrame*> KeyFrame::GetCovisiblesByWeight(const int &w)
{
    unique_lock<mutex> lockCon(mMutexConnections);

    if (mvpOrderedConnectedKeyFrames.empty())
        return vector<KeyFrame*>();

    vector<int>::iterator it = upper_bound(mvOrderedWeights.begin(), mvOrderedWeights.end(), w, KeyFrame::weightComp);

    if (it == mvOrderedWeights.end())
    {
        return vector<KeyFrame*>();
    }
    else
    {
        int n = it - mvOrderedWeights.begin();
        return vector<KeyFrame*>(mvpOrderedConnectedKeyFrames.begin(), mvpOrderedConnectedKeyFrames.begin() + n);
    }
}

} // namespace PL_VO