//
// Created by rain on 18-1-25.
//

#ifndef PL_VO_KEYFRAME_H
#define PL_VO_KEYFRAME_H

#include <glog/logging.h>
#include "Frame.h"
#include "Map.h"

namespace PL_VO
{

class MapPoint;
class MapLine;
class Frame;
class Map;

class KeyFrame
{
public:

    KeyFrame(Frame &frame, Map *pMap);

    size_t GetFrameID();

    void AddConnection(KeyFrame* pKeyFrame, const int &weight);

    void UpdateBestCovisibles();

    void AddChild(KeyFrame* pKeyFrame);

    void UpdateConnections();


    Camera *mpCamera = nullptr;

    double mtimeStamp;
    Sophus::SE3 Tcw;
    Sophus::SE3 Twc;

    vector<cv::KeyPoint> mvKeyPoint;
    vector<cv::line_descriptor::KeyLine> mvKeyLine;
    vector<cv::KeyPoint> mvKeyPointUn;
    vector<cv::line_descriptor::KeyLine> mvKeyLineUn;
    vector<LineFeature2D*> mvpLineFeature2D;
    vector<PointFeature2D*> mvpPointFeature2D;
    vector<MapPoint*> mvpMapPoint;
    vector<MapLine*> mvpMapLine;

    map<KeyFrame*, int> mConnectedKeyFrameWeights;
    vector<KeyFrame*> mvpOrderedConnectedKeyFrames;
    vector<int> mvOrderedWeights;

    bool mbFirstConnection;
    KeyFrame* mpParent;
    set<KeyFrame*> mspChildrens;
    set<KeyFrame*> mspLoopEdges;

private:

    size_t mID;
    mutex mMutexPose;
    mutex mMutexConnections;
    mutex mMutexFeatures;

}; // class KeyFrame

} // namespace PL_VO


#endif //PL_VO_KEYFRAME_H
