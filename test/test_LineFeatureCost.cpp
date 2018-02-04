//
// Created by rain on 18-1-23.
//

#include "Optimizer.h"
#include "Map.h"

int main(int argc, char *argv[])
{
    PL_VO::MapLine *pMapLine = new(PL_VO::MapLine);
    PL_VO::Frame *pFrame;
    PL_VO::Camera *pCamera = new PL_VO::Camera("../Example/TUM2.yaml");

    Sophus::SE3d Tcw;

    Tcw.setQuaternion(Eigen::Quaterniond(0.99998091547793622, 0.004081818193394318, 0.0036343218945305089, 0.0028808235821990726));
    Tcw.translation() =  Eigen::Vector3d(-0.00040315662988363505, -0.002429691464738472,  0.005346058084408755);

//    pMapLine->mPoseStartw = Eigen::Vector3d(-0.83358351561463107, -0.065460973511881085, 1.799638211339911);
//    pMapLine->mPoseEndw = Eigen::Vector3d(-0.83963883022847785, -0.11686640926557536, 1.9030361891241583);

    pMapLine->mPoseStartw = Eigen::Vector3d(0.31902656360042397, 0.13819605303346758, 1.1448);
    pMapLine->mPoseEndw = Eigen::Vector3d(0.32675167720338882, 0.19607757641444701, 1.1317999999999999);

    PL_VO::LineFeature2D *pLineFeature2D = new PL_VO::LineFeature2D(Eigen::Vector2d(185.06536865234375, 418.77471923828125),
                                                                    Eigen::Vector2d(174.42501831054688, 437.40206909179688));

    pMapLine->mmpLineFeature2D[0] = pLineFeature2D;
    pMapLine->mmpLineFeature2D[1] = pLineFeature2D;
    pMapLine->mmpLineFeature2D[2] = pLineFeature2D;

    cout << PL_VO::Optimizer::ComputeMapLineCost(pMapLine, Tcw.unit_quaternion(), Tcw.translation(), pCamera->GetCameraIntrinsic(), 1);

    return 0;
}

