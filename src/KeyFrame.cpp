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
    Tcw = frame.Tcw;
    Twc = frame.Twc;

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
    return mID;
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
        LOG_IF(ERROR, KFcounter.empty()) << "no MapPoint and LinePoint between the current KeyFrame with the other KeyFrame" << endl;
        return;
    }

    int nmax = 0;
    Frame *pFrameMax = nullptr;
    int th = 30;

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

bool KeyFrame::isBad()
{
    unique_lock<mutex> lock(mMutexConnections);
    return mbBad;
}

} // namespace PL_VO