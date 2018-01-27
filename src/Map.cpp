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


MapLine::MapLine()
{
}

vector<Frame*> MapLine::GetObservedFrame()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mvpFrameinvert;
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
