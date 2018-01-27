//
// Created by rain on 18-1-26.
//

#ifndef PL_VO_LOCALMAPPING_H
#define PL_VO_LOCALMAPPING_H

#include "KeyFrame.h"

namespace PL_VO
{

class Tracking;
class Map;

class LocalMapping
{

public:

    LocalMapping(Map *pMap);

    void SetTracking(Tracking *pTracker);

    void Run();

    void InsertKeyFrame(KeyFrame *pKeyFrame);

    void ProcessNewKeyFrame();

private:

    bool CheckNewKeyFrames();

    Tracking *mpTracker;
    Map *mpMap;

    list<KeyFrame*> mlpNewKeyFrames;

    KeyFrame *mpCurrentKeyFrame;

    mutex mMuteNewKFs;
}; // class LocalMapping

} // namespace PL_VO



#endif //PL_VO_LOCALMAPPING_H
