//
// Created by rain on 18-1-2.
//

#include "Map.h"

namespace PL_VO
{

MapPoint::MapPoint()
{
}

vector<Frame*> MapPoint::GetObservedFrame()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mvpFrameinvert;
}

Eigen::Vector3d MapPoint::GetPose()
{
    unique_lock<mutex> lock(mMutexPos);
    return mPosew;
}

bool MapPoint::isBad()
{
    unique_lock<mutex> lock(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
    return mbBad;
}

void MapPoint::SetBadFlag()
{
    unique_lock<mutex> lock1(mMutexFeatures);
    mbBad = true;

    // TODO clear this MapPoint in the keyframe which includes this MapPoint
}

MapLine::MapLine()
{
}

vector<Frame*> MapLine::GetObservedFrame()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mvpFrameinvert;
}

Eigen::Vector3d MapLine::GetPoseStart()
{
    unique_lock<mutex> lock(mMutexPos);
    return mPoseStartw;
}

Eigen::Vector3d MapLine::GetPoseEnd()
{
    unique_lock<mutex> lock(mMutexPos);
    return mPoseEndw;
}

bool MapLine::isBad()
{
    unique_lock<mutex> lock(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
    return mbBad;
}

void MapLine::SetBadFlag()
{
    unique_lock<mutex> lock1(mMutexFeatures);
    mbBad = true;

    // TODO clear this MapLine in the keyframe which includes this MapLine
}

Map::Map()
{
}

void Map::AddKeyFrame(KeyFrame *pKeyFrame)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrame.insert(pKeyFrame);
}

void Map::AddMapPoint(MapPoint *pMapPoint)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoint.insert(pMapPoint);
}

void Map::AddMapLine(MapLine *pMapLine)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapLine.insert(pMapLine);
}

void Map::EraseKeyFrame(KeyFrame *pKeyFrame)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrame.erase(pKeyFrame);
}

void Map::EraseMapPoint(MapPoint *pMapPoint)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoint.erase(pMapPoint);
}

void Map::EraseMapLine(MapLine *pMapLine)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapLine.erase(pMapLine);
}

vector<KeyFrame*> Map::GetAllKeyFrames()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<KeyFrame*>(mspKeyFrame.begin(), mspKeyFrame.end());
}

const vector<KeyFrame *> Map::GetAllKeyFrames() const
{
    return vector<KeyFrame*>(mspKeyFrame.begin(), mspKeyFrame.end());
}

vector<MapPoint*> Map::GetAllMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<MapPoint*>(mspMapPoint.begin(), mspMapPoint.end());
}

vector<MapLine*> Map::GetAllMapLines()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<MapLine*>(mspMapLine.begin(), mspMapLine.end());
}


} // namespace PL_VO
