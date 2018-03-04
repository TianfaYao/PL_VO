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

    double FindDepth(const cv::Point2f &point, const cv::Mat &imagedepth);

    void UnprojectStereo(const cv::Mat &imageDepth, const vector<cv::DMatch> vpointMatches,
                         const vector<cv::DMatch> vlineMatches);

    void AddConnection(KeyFrame* pKeyFrame, const int &weight);

    void UpdateBestCovisibles();

    void AddChild(KeyFrame* pKeyFrame);

    void UpdateConnections();

    vector<KeyFrame*> GetVectorCovisibleKeyFrames();

    vector<MapPoint*> GetMapPointMatches();

    vector<MapLine*> GetMapLineMatches();

    Sophus::SE3d GetPose();

    Eigen::Vector3d GetCameraCenter();

    void SetPose(Sophus::SE3d Tcw_);

    bool isBad();

    KeyFrame* GetParent();

    vector<KeyFrame*> GetCovisiblesByWeight(const int &w);

    Camera *mpCamera = nullptr;
    Map *mpMap = nullptr;

    double mtimeStamp;

    Sophus::SE3d Tcw;
    Sophus::SE3d Twc;
    Sophus::SE3d TcwOptimize;

    cv::Mat mpointDesc;
    cv::Mat mlineDesc;

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
    Frame *mpFrame = nullptr;
    KeyFrame* mpParent = nullptr;
    set<KeyFrame*> mspChildrens;
    set<KeyFrame*> mspLoopEdges;

    size_t mBALocalForKF;
    size_t mBAFixedForKF;

    static bool weightComp( int a, int b)
    {
        return a>b;
    }

private:

    void UnprojectPointStereo(const cv::Mat &imageDepth, const vector<cv::DMatch> &vpointMatches);

    void UnprojectLineStereo(const cv::Mat &imageDepth, const vector<cv::DMatch> &vlineMatches);

    bool mbBad;

    int mImageHeight;
    int mImageWidth;

    size_t mID;
    mutex mMutexPose;
    mutex mMutexConnections;
    mutex mMutexFeatures;

}; // class KeyFrame

} // namespace PL_VO


#endif //PL_VO_KEYFRAME_H
