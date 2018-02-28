//
// Created by rain on 18-2-27.
//

#ifndef PL_VO_VIEWER_H
#define PL_VO_VIEWER_H

#include <iostream>
#include <string>
#include <vector>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <pangolin/pangolin.h>
#include <glog/logging.h>

#include "Tracking.h"
#include "MapDrawer.h"

using namespace std;

namespace PL_VO
{

class Tracking;
class MapDrawer;

class Viewer
{

public:
    Viewer(const string &strSettingPath, MapDrawer* pMapDrawer);

    void UpdateShowImage(Tracking *pTracking);

    void Run();

    void RequestFinish();

    void RequestStop();

    bool isFinished();

    bool isStopped();

    void Release();

private:

    bool Stop();

    bool CheckFinish();

    void SetFinish();

    Tracking *mpTracking = nullptr;
    MapDrawer *mpMapDrawer = nullptr;

    cv::Mat mImageShow;

    double mImageHeight;
    double mImageWidth;

    float mViewpointX, mViewpointY, mViewpointZ, mViewpointF;

    bool mbFinishRequested = false;
    bool mbFinished = true;
    bool mbStopped = true;
    bool mbStopRequested = false;

    mutex mMutex;
    mutex mMutexStop;
    mutex mMutexFinish;

}; // class Viewer

} // namespace PL_VO


#endif //PL_VO_VIEWER_H
