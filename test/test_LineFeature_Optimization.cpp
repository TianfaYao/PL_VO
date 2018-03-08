//
// Created by rain on 18-3-6.
//

#include <Camera.h>
#include <Converter.h>
#include "Frame.h"
#include "Optimizer.h"
#include "Tracking.h"
#include "Map.h"
#include "CVUtils.h"

using namespace std;


int main(int argc, char *argv[])
{
    const int posenum = 2;
    const int linenum = 100;
    const int pointnum = 100;
    const double PIXELNOISE = 0.1;
    const double MapPLNOISE = 0.5;

    PL_VO::Camera *pCamera;
    PL_VO::Map *pMap;
    PL_VO::Frame *pcurFrame;
    PL_VO::PointFeature *ppointFeaturer;
    PL_VO::LineFeature *plineFeaturer;

    plineFeaturer = new PL_VO::LineFeature;
    ppointFeaturer = new PL_VO::PointFeature;
    pCamera = new PL_VO::Camera("../Example/TUM2.yaml");
    pMap = new PL_VO::Map;

    vector<Sophus::SE3d> vTrueStates;
    vector<PL_VO::Frame *> vpFrmaes;
    vector<PL_VO::MapLine *> vpMapLines;
    vector<pair<Eigen::Vector3d, Eigen::Vector3d>> vTrueMapLines;
    vector<Eigen::Vector3d> vTrueMapPoints;

    for (int i = 0; i < posenum; i++)
    {
        pcurFrame = new PL_VO::Frame(i*0.1, pCamera, plineFeaturer, ppointFeaturer);

        Eigen::Vector3d noise3d = Eigen::Vector3d(PL_VO::Sample::gaussian(0.1), PL_VO::Sample::gaussian(0.1), PL_VO::Sample::gaussian(0.1));

        Eigen::Vector3d twc(i*0.04 - 1.0, 0., 0.);
        Eigen::Quaterniond Rwc;
        Rwc.setIdentity();

        vTrueStates.emplace_back(Sophus::SE3d(Rwc, twc));
        pcurFrame->SetPose(Sophus::SE3d(Rwc, twc+noise3d));
        vpFrmaes.emplace_back(pcurFrame);

        cout << "the true pose: " ;
        cout << twc.transpose() << endl;
    }

    for (int i = 0; i < pointnum; i++)
    {
        PL_VO::MapPoint *pMapPoint = new PL_VO::MapPoint();

        pMapPoint->mID = (size_t)i;

        Eigen::Vector3d PointNoise3d = Eigen::Vector3d(PL_VO::Sample::gaussian(MapPLNOISE),
                                                       PL_VO::Sample::gaussian(MapPLNOISE),
                                                       PL_VO::Sample::gaussian(MapPLNOISE));

        Eigen::Vector3d Pose = Eigen::Vector3d((PL_VO::Sample::uniform()-1)*3,
                                                PL_VO::Sample::uniform()-1,
                                                PL_VO::Sample::uniform()+2);

        vTrueMapPoints.emplace_back(Pose);

        cout << i << " : " << Pose.transpose() << endl;

        pMapPoint->SetPose(Pose);

        for (int j = 0; j < posenum; j++)
        {
            PL_VO::PointFeature2D *pPointFeature2D;
            Eigen::Vector2d pixel;
            Eigen::Quaterniond R = vpFrmaes[j]->GetPose().unit_quaternion();
            Eigen::Vector3d t = vpFrmaes[j]->GetPose().translation();

            pixel = pCamera->World2Pixel(Pose, R, t);

            if (!pCamera->inBorder(pixel))
                continue;

            pPointFeature2D = new PL_VO::PointFeature2D(pixel);

            pMapPoint->mmpPointFeature2D[vpFrmaes[j]->GetFrameID()] = pPointFeature2D;

            vpFrmaes[j]->mvpMapPoint.emplace_back(pMapPoint);
        }
    }

    for (int i = 0; i < linenum; i++)
    {
        PL_VO::MapLine *pMapLine = new PL_VO::MapLine();

        pMapLine->mID = (size_t)i;

        Eigen::Vector3d PoseStartNoise3d = Eigen::Vector3d(PL_VO::Sample::gaussian(MapPLNOISE),
                                                           PL_VO::Sample::gaussian(MapPLNOISE),
                                                           PL_VO::Sample::gaussian(MapPLNOISE));

        Eigen::Vector3d PoseEndNoise3d = Eigen::Vector3d(PL_VO::Sample::gaussian(MapPLNOISE),
                                                         PL_VO::Sample::gaussian(MapPLNOISE),
                                                         PL_VO::Sample::gaussian(MapPLNOISE));

        Eigen::Vector3d PoseStart = Eigen::Vector3d((PL_VO::Sample::uniform()-0.5)*3,
                                                     PL_VO::Sample::uniform()-0.5,
                                                     PL_VO::Sample::uniform()+3);

        Eigen::Vector3d PoseEnd = Eigen::Vector3d((PL_VO::Sample::uniform()-0.5)*3,
                                                   PL_VO::Sample::uniform()-0.5,
                                                   PL_VO::Sample::uniform()+3);

        vTrueMapLines.emplace_back(make_pair(PoseStart, PoseEnd));

        cout << i << " : " << PoseStart.transpose() << " | " << PoseEnd.transpose() << endl;

        pMapLine->SetPose(PoseStart+PoseStartNoise3d, PoseEnd+PoseEndNoise3d);

        for (int j = 0; j < posenum; j++)
        {
            PL_VO::LineFeature2D *pLineFeature2D;
            Eigen::Vector2d Startpixel, Endpixel;

            Eigen::Quaterniond R = vpFrmaes[j]->GetPose().unit_quaternion();
            Eigen::Vector3d t = vpFrmaes[j]->GetPose().translation();

            Startpixel = pCamera->World2Pixel(PoseStart, R, t);
            Endpixel = pCamera->World2Pixel(PoseEnd, R, t);

            if (!pCamera->inBorder(Startpixel) || !pCamera->inBorder(Endpixel))
                continue;

            pLineFeature2D = new PL_VO::LineFeature2D(Startpixel, Endpixel);

//            cout << "the LineFeature2D: " << endl;
//            cout << Startpixel.transpose() << " | " << Endpixel.transpose() << endl;

            pMapLine->mmpLineFeature2D[vpFrmaes[j]->GetFrameID()] = pLineFeature2D;

            vpFrmaes[j]->mvpMapLine.emplace_back(pMapLine);
        }
    }

    PL_VO::KeyFrame *pKeyFrame;
    pKeyFrame = new PL_VO::KeyFrame(*vpFrmaes[0], pMap);
    pKeyFrame->mpFrame = vpFrmaes[0];

    PL_VO::Optimizer::PoseOptimization(vpFrmaes[1], pKeyFrame);


    return 1;
}

