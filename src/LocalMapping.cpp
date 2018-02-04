//
// Created by rain on 18-1-26.
//

#include "LocalMapping.h"

namespace PL_VO
{

LocalMapping::LocalMapping(Map *pMap)
{
    mpMap = pMap;
}

void LocalMapping::SetTracking(Tracking *pTracker)
{
    mpTracker = pTracker;
}

void LocalMapping::Run()
{
    while (1)
    {
        if (CheckNewKeyFrames())
        {
            ProcessNewKeyFrame();

            bool bstopflag = true;
            Optimizer::LocalBundleAdjustment(mpCurrentKeyFrame, &bstopflag, mpMap);
        }
        usleep(3000);
    }
}

void LocalMapping::InsertKeyFrame(KeyFrame *pKeyFrame)
{
    unique_lock<mutex> lock(mMuteNewKFs);
    mlpNewKeyFrames.push_back(pKeyFrame);
}

bool LocalMapping::CheckNewKeyFrames()
{
    unique_lock<mutex> lock(mMuteNewKFs);
    return(!mlpNewKeyFrames.empty());
}

void LocalMapping::ProcessNewKeyFrame()
{
    {
        unique_lock<mutex> lock(mMuteNewKFs);
        mpCurrentKeyFrame = mlpNewKeyFrames.front();
        mlpNewKeyFrames.pop_front();
    }

    mpCurrentKeyFrame->UpdateConnections();

    mpMap->AddKeyFrame(mpCurrentKeyFrame);
}



} // namespace PL_VO