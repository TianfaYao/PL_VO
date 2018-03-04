//
// Created by rain on 18-2-27.
//

#include "MapDrawer.h"

namespace PL_VO
{
MapDrawer::MapDrawer(Map* pMap, const string &strSettingPath):mpMap(pMap)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    if (!fSettings.isOpened())
    {
        LOG(FATAL) << "Failed to open settings file at " << strSettingPath << endl;
        exit(-1);
    }

    mKeyFrameSize = fSettings["Viewer.KeyFrameSize"];
    mKeyFrameLineWidth = fSettings["Viewer.KeyFrameLineWidth"];
    mGraphLineWidth = fSettings["Viewer.GraphLineWidth"];
    mPointSize = fSettings["Viewer.PointSize"];
    mCameraSize = fSettings["Viewer.CameraSize"];
    mCameraLineWidth = fSettings["Viewer.CameraLineWidth"];

    mCameraPose.setIdentity();
}

/**
* @brief use this transformation can directly to display
* @param Twc from the camere to world
* @return the pangolin opengl matrix type
*/
pangolin::OpenGlMatrix MapDrawer::toOpenGLMatrix(const Eigen::Matrix<double, 3, 4> &Twc)
{
    pangolin::OpenGlMatrix M;

    Eigen::Matrix3d Rwc = Twc.block<3, 3>(0, 0);
    Eigen::Vector3d twc = Twc.block<3, 1>(0, 3);

    M.m[0] = Rwc(0,0);
    M.m[1] = Rwc(1,0);
    M.m[2] = Rwc(2,0);
    M.m[3] = 0.0;

    M.m[4] = Rwc(0,1);
    M.m[5] = Rwc(1,1);
    M.m[6] = Rwc(2,1);
    M.m[7] = 0.0;

    M.m[8] = Rwc(0,2);
    M.m[9] = Rwc(1,2);
    M.m[10] = Rwc(2,2);
    M.m[11] = 0.0;

    M.m[12] = twc(0);
    M.m[13] = twc(1);
    M.m[14] = twc(2);
    M.m[15] = 1.0;

    return M;
}

void MapDrawer::DrawMapPoints()
{
    const vector<MapPoint*> &vpMPs = mpMap->GetAllMapPoints();

    if (vpMPs.empty())
        return;

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(0.0,0.0,0.0);

    for (auto pMP : vpMPs)
    {
        if (pMP->isBad())
            continue;

        Eigen::Vector3d pos = pMP->GetPose();
        glVertex3f(pos[0], pos[1], pos[2]);
    }

    glEnd();
}

void MapDrawer::DrawMapLines()
{
    const vector<MapLine*> &vpMLs = mpMap->GetAllMapLines();

    if (vpMLs.empty())
        return;

    glLineWidth(mGraphLineWidth);
    glColor4f(0.0f,1.0f,0.0f,0.6f);
    glBegin(GL_LINES);

    for (auto pML : vpMLs)
    {
        if (pML->isBad())
            continue;

        Eigen::Vector3d posStart = pML->GetPoseStart();
        Eigen::Vector3d posEnd = pML->GetPoseEnd();

        glVertex3f((float)posStart[0], (float)posStart[1], (float)posStart[2]);
        glVertex3f((float)posEnd[0], (float)posEnd[1], (float)posEnd[2]);
    }

    glEnd();
}

// from the camera to world
void MapDrawer::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc)
{
    const float &w = mCameraSize; // width
    const float h = w*0.75; // height
    const float z = w*0.6; // z axis

    glPushMatrix();

#ifdef HAVE_GLES
    glMultMatrixf(Twc.m);
#else
    glMultMatrixd(Twc.m);
#endif

    glLineWidth(mCameraLineWidth);
    glColor3f(0.0f,1.0f,0.0f);
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(w,h,z);
    glVertex3f(0,0,0);
    glVertex3f(w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,h,z);

    glVertex3f(w,h,z);
    glVertex3f(w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(-w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(w,h,z);

    glVertex3f(-w,-h,z);
    glVertex3f(w,-h,z);
    glEnd();

    glPopMatrix();
}

void MapDrawer::DrawKeyFrames(const bool &bDrawKF, const bool &bDrawGraph)
{
    const float &w = mCameraSize;
    const float h = w*0.75;
    const float z = w*0.6;

    const vector<KeyFrame *> vpKFs = mpMap->GetAllKeyFrames();

    if (bDrawKF)
    {
        for (size_t i = 0; i < vpKFs.size(); i++)
        {
            KeyFrame* pKF = vpKFs[i];
            Eigen::Matrix<double, 3, 4> Twc = pKF->GetPose().matrix3x4();

            Eigen::Matrix3d Rwc = Twc.block<3, 3>(0, 0);
            Eigen::Vector3d twc = Twc.block<3, 1>(0, 3);

            Twc.block<3, 3>(0, 0) = Rwc.inverse();
            Twc.block<3, 1>(0, 3) = -Rwc.inverse()*twc;

            pangolin::OpenGlMatrix M;

            M = toOpenGLMatrix(Twc);

            glPushMatrix();

            glMultMatrixd(M.m);

            glLineWidth(mKeyFrameLineWidth);
            glColor3f(0.0f,0.0f,1.0f);
            glBegin(GL_LINES);
            glVertex3f(0,0,0);
            glVertex3f(w,h,z);
            glVertex3f(0,0,0);
            glVertex3f(w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,h,z);

            glVertex3f(w,h,z);
            glVertex3f(w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(-w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(w,h,z);

            glVertex3f(-w,-h,z);
            glVertex3f(w,-h,z);
            glEnd();

            glPopMatrix();
        }
    }

    if (bDrawGraph)
    {
        for (size_t i = 0; i <  vpKFs.size(); i++)
        {
            // Covisiblity Graph, show the connection of the keyframes which can see the common view
            const vector<KeyFrame*> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);

            Eigen::Vector3d Ow = vpKFs[i]->GetCameraCenter();

            if (!vCovKFs.empty())
            {
                for (auto vit : vCovKFs)
                {
                    if (vit->GetFrameID() < vpKFs[i]->GetFrameID())
                        continue;

                    Eigen::Vector3d Ow2 = vit->GetCameraCenter();

                    glLineWidth(mGraphLineWidth);
                    glColor4f(0.0f,1.0f,0.0f,0.6f);
                    glBegin(GL_LINES);
                    glVertex3f((float)Ow[0], (float)Ow[1], (float)Ow[2]);
                    glVertex3f((float)Ow2[0], (float)Ow2[1], (float)Ow2[2]);
                    glEnd();
                }
            }

            // Spanning tree
            KeyFrame *pParent = vpKFs[i]->GetParent();

            if (pParent)
            {
                glLineWidth(mGraphLineWidth+2);
                glColor4f(0.0f,0.0f,0.0f,1.0f);
                glBegin(GL_LINES);
                Eigen::Vector3d Owp = pParent->GetCameraCenter();
                glVertex3f((float)Ow[0], (float)Ow[1], (float)Ow[2]);
                glVertex3f((float)Owp[0], (float)Owp[1], (float)Owp[2]);
                glEnd();
            }
        }

//        glEnd();
    }
}

// from the wrold to camera
void MapDrawer::SetCurrentCameraPose(const Eigen::Matrix<double, 3, 4> &Twc)
{
    unique_lock<mutex> lock(mMutexCamera);
    mCameraPose = Twc;
}

void MapDrawer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M)
{
    M = toOpenGLMatrix(mCameraPose);
}

} // namespace PL_VO
