//
// Created by rain on 18-1-3.
//

#include <KeyFrame.h>
#include "Optimizer.h"

namespace PL_VO
{

bool PoseLocalParameterization::Plus(const double *x, const double *delta, double *xplusdelta) const
{
    // x is the seven dimensional space
    Eigen::Map<const Eigen::Vector3d> trans(x + 4);
    Eigen::Map<const Eigen::Quaterniond> quaterd(x);

    // delta is the parameter in the parameter space
    // delta is the six dimensional space
    Sophus::SE3d se3delta = Sophus::SE3d::exp(Eigen::Map<const Eigen::Matrix<double, 6, 1, Eigen::ColMajor>>(delta));

    Eigen::Map<Eigen::Quaterniond> quaterdplus(xplusdelta);
    Eigen::Map<Eigen::Vector3d> transplus(xplusdelta + 4);

    quaterdplus = se3delta.so3().matrix() * quaterd;
    transplus = se3delta.so3().matrix() * trans + se3delta.translation();

    return true;
}


bool PoseLocalParameterization::ComputeJacobian(const double *x, double *jacobian) const
{
    Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor> > J(jacobian);
    J.setZero();
    J.block<6,6>(0, 0).setIdentity();
    return true;
}

/**
 * @brief rotation(quaternion), translation(vector3d), point3d(vector3d)
 * @param parameters
 * @param residuals
 * @param jacobians
 * @return
 */
bool ReprojectionErrorSE3::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    Eigen::Map<const Eigen::Quaterniond> quaterd(parameters[0]);
    Eigen::Map<const Eigen::Vector3d> trans(parameters[0] + 4);
    Eigen::Map<const Eigen::Vector3d> point(parameters[1]);
    Eigen::Map<Eigen::Vector2d> residual(residuals);

    Eigen::Vector3d p = quaterd * point + trans;

    residual[0] = fx*p[0]/p[2] + cx - observedx;
    residual[1] = fy*p[1]/p[2] + cy - observedy;

//    CHECK(residual[0] < 100);

//    residual = sqrtInforMatrix*residual;

    Eigen::Matrix<double, 2, 3, Eigen::RowMajor> jacobian;

    jacobian << fx/p[2],  0, -fx*p[0]/p[2]/p[2],
                 0, fy/p[2], -fy*p[1]/p[2]/p[2];

    if(jacobians != nullptr)
    {
        if(jacobians[0] != nullptr) // the jacobian for the MapPoint pose optimization
        {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor> > Jse3(jacobians[0]);
            Jse3.setZero();

            // very important! the form of the se3 is the rotation in the front and the transformation in the back
            Jse3.block<2,3>(0,0) = jacobian;
            Jse3.block<2,3>(0,3) = -jacobian*Converter::skew(p);

//            Jse3 = sqrtInforMatrix*Jse3;

//            CHECK(fabs(Jse3.maxCoeff()) < 1e8);
//            CHECK(fabs(Jse3.minCoeff()) < 1e8);
        }
        if(jacobians[1] != nullptr) // the jacobian for the camera pose optimization
        {
            Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor> > Jpoint(jacobians[1]);
            Jpoint = jacobian * quaterd.toRotationMatrix();

//           Jpoint = sqrtInforMatrix*Jpoint;

//            CHECK(fabs(Jpoint.maxCoeff()) < 1e8);
//            CHECK(fabs(Jpoint.minCoeff()) < 1e8);
        }
    }

    return true;
}

bool ReprojectionLineErrorSE3::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    Eigen::Map<const Eigen::Quaterniond> quaterd(parameters[0]);
    Eigen::Map<const Eigen::Vector3d> trans(parameters[0] + 4);
    Eigen::Map<const Eigen::Vector3d> Startpoint3d(parameters[1]);
    Eigen::Map<const Eigen::Vector3d> Endpoint3d(parameters[2]);
    Eigen::Map<Eigen::Vector2d> residual(residuals);

    Eigen::Vector3d Startpoint3dC;
    Eigen::Vector3d Endpoint3dC;
    Eigen::Vector2d Startpoint2d;
    Eigen::Vector2d Endpoint2d;
    Eigen::Vector2d err;

    Startpoint3dC = quaterd*Startpoint3d + trans;
    Endpoint3dC = quaterd*Endpoint3d + trans;

    Startpoint2d[0] = fx*Startpoint3dC[0]/Startpoint3dC[2] + cx;
    Startpoint2d[1] = fy*Startpoint3dC[1]/Startpoint3dC[2] + cy;

    Endpoint2d[0] = fx*Endpoint3dC[0]/Endpoint3dC[2] + cx;
    Endpoint2d[1] = fy*Endpoint3dC[1]/Endpoint3dC[2] + cy;

    err[0] = lineCoef[0]*Startpoint2d[0] + lineCoef[1]*Startpoint2d[1] + lineCoef[2];
    err[1] = lineCoef[0]*Endpoint2d[0] + lineCoef[1]*Endpoint2d[1] + lineCoef[2];

    residual[0] = err[0];
    residual[1] = err[1];

    Eigen::Matrix<double, 2, 3, Eigen::RowMajor> jacobianStart;
    Eigen::Matrix<double, 2, 3, Eigen::RowMajor> jacobianEnd;
    Eigen::Matrix<double, 2, 3, Eigen::RowMajor> jacobian;

    jacobianStart << fx/Startpoint3dC[2],  0, -fx*Startpoint3dC[0]/Startpoint3dC[2]/Startpoint3dC[2],
                      0, fy/Startpoint3dC[2], -fy*Startpoint3dC[1]/Startpoint3dC[2]/Startpoint3dC[2];

    jacobianEnd << fx/Endpoint3dC[2],  0, -fx*Endpoint3dC[0]/Endpoint3dC[2]/Endpoint3dC[2],
                    0, fy/Endpoint3dC[2], -fy*Endpoint3dC[1]/Endpoint3dC[2]/Endpoint3dC[2];

//    residual = sqrtInforMatrix*residual;

    if (jacobians != nullptr)
    {
        if(jacobians[0] != nullptr) // the jacobian for the MapPoint pose optimization
        {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor> > Jse3(jacobians[0]);
            Jse3.setZero();

            // very important! the form of the se3 is the rotation in the front and the transformation in the back
            jacobian(0, 0) = lineCoef[0]*jacobianStart(0, 0);
            jacobian(0, 1) = lineCoef[1]*jacobianStart(1, 1);
            jacobian(0, 2) = lineCoef[0]*jacobianStart(0, 2) + lineCoef[1]*jacobianStart(1, 2);
            jacobian(1, 0) = lineCoef[0]*jacobianEnd(0, 0);
            jacobian(1, 1) = lineCoef[1]*jacobianEnd(1, 1);
            jacobian(1, 2) = lineCoef[0]*jacobianEnd(0, 2) + lineCoef[1]*jacobianEnd(1, 2);

            Jse3.block<2, 3>(0, 0) = jacobian;
            Jse3.block<1, 3>(0, 3) = -jacobian.block<1, 3>(0, 0)*Converter::skew(Startpoint3dC);
            Jse3.block<1, 3>(1, 3) = -jacobian.block<1, 3>(1, 0)*Converter::skew(Endpoint3dC);

//            Jse3 = sqrtInforMatrix*Jse3;

//            CHECK(fabs(Jse3.maxCoeff()) < 1e8);
//            CHECK(fabs(Jse3.minCoeff()) < 1e8);
        }

        if(jacobians[1] != nullptr) // the jacobian for the camera pose optimization
        {
            Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor> > Jpoint(jacobians[1]);
            Jpoint.setZero();

            Jpoint.block<1, 3>(0, 0) = jacobian.block<1, 3>(0 ,0)*quaterd.toRotationMatrix();

//            Jpoint = sqrtInforMatrix*Jpoint;

//            CHECK(fabs(Jpoint.maxCoeff()) < 1e8);
//            CHECK(fabs(Jpoint.minCoeff()) < 1e8);
        }

        if (jacobians[2] != nullptr)
        {
            Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor> > Jpoint(jacobians[2]);
            Jpoint.setZero();

            Jpoint.block<1, 3>(1, 0) = jacobian.block<1, 3>(1 ,0)*quaterd.toRotationMatrix();

//            Jpoint = sqrtInforMatrix*Jpoint;

//            CHECK(fabs(Jpoint.maxCoeff()) < 1e8);
//            CHECK(fabs(Jpoint.minCoeff()) < 1e8);
        }

    }

    return true;

}

double Optimizer::ComputeMapLineCost(MapLine *pMapLine, const Eigen::Quaterniond R, const Eigen::Vector3d t,
                                     const Eigen::Matrix3d K, size_t observeID)
{
    double fx, fy, cx, cy;
    double cost;
    fx = K(0, 0);
    fy = K(1, 1);
    cx = K(0, 2);
    cy = K(1, 2);

    Eigen::Vector3d Startpoint3dC;
    Eigen::Vector3d Endpoint3dC;
    Eigen::Vector2d Startpoint2d;
    Eigen::Vector2d Endpoint2d;
    Eigen::Vector2d err;
    Eigen::Vector3d lineCoef;

    lineCoef = pMapLine->mmpLineFeature2D[observeID]->mLineCoef;

    Startpoint3dC = R*pMapLine->mPoseStartw + t;
    Endpoint3dC = R*pMapLine->mPoseEndw + t;

    Startpoint2d[0] = fx*Startpoint3dC[0]/Startpoint3dC[2] + cx;
    Startpoint2d[1] = fy*Startpoint3dC[1]/Startpoint3dC[2] + cy;

    Endpoint2d[0] = fx*Endpoint3dC[0]/Endpoint3dC[2] + cx;
    Endpoint2d[1] = fy*Endpoint3dC[1]/Endpoint3dC[2] + cy;

    err[0] = lineCoef[0]*Startpoint2d[0] + lineCoef[1]*Startpoint2d[1] + lineCoef[2];
    err[1] = lineCoef[0]*Endpoint2d[0] + lineCoef[1]*Endpoint2d[1] + lineCoef[2];

    cost = err.norm();

    return cost;
}

double Optimizer::ComputeMapPointCost(MapPoint *pMapPoint, const Eigen::Quaterniond R, const Eigen::Vector3d t,
                                      const Eigen::Matrix3d K, size_t observeID)
{
    Eigen::Vector2d err;
    Eigen::Vector2d predicted;
    double fx, fy, cx, cy;
    double cost;
    fx = K(0, 0);
    fy = K(1, 1);
    cx = K(0, 2);
    cy = K(1, 2);

    Eigen::Vector3d p = R*pMapPoint->GetPose() + t;

    predicted[0] = fx*p[0]/p[2] + cx;
    predicted[1] = fy*p[1]/p[2] + cy;

    double temp = pMapPoint->mmpPointFeature2D[observeID]->mpixel[0];

    err[0] = predicted[0]- pMapPoint->mmpPointFeature2D[observeID]->mpixel[0];
    err[1] = predicted[1]- pMapPoint->mmpPointFeature2D[observeID]->mpixel[1];

    cost = err.norm();

    return cost;
}


Eigen::Vector2d Optimizer::ReprojectionError(const ceres::Problem& problem, ceres::ResidualBlockId id)
{
    auto cost = problem.GetCostFunctionForResidualBlock(id);

    std::vector<double*> parameterBlocks;
    problem.GetParameterBlocksForResidualBlock(id, &parameterBlocks);

    Eigen::Vector2d residual;
    cost->Evaluate(parameterBlocks.data(), residual.data(), nullptr);

    return residual;
}

vector<double> Optimizer::GetReprojectionErrorNorms(const ceres::Problem& problem)
{
    std::vector<double> result;
    std::vector<ceres::ResidualBlockId> ids;

    problem.GetResidualBlocks(&ids);

    for (auto &id : ids)
    {
        result.push_back(ReprojectionError(problem, id).norm());
    }

    return result;
}

void Optimizer::GetPLReprojectionErrorNorms(const ceres::Problem &problem, const double *pPointParameter,
                                            const double *pLineParameter, vector<double> &vPointResidues,
                                            vector<double> &vLineResidues)
{
    std::vector<ceres::ResidualBlockId> vidsPoint;
    std::vector<ceres::ResidualBlockId> vidsLine;
    std::vector<ceres::ResidualBlockId> vidsPL;

    problem.GetResidualBlocks(&vidsPL);

//    problem.GetResidualBlocksForParameterBlock(pPointParameter, &vidsPoint);
    problem.GetResidualBlocksForParameterBlock(pLineParameter, &vidsLine);

    auto it = find(vidsPL.begin(), vidsPL.end(), vidsLine[0]);
    if (it == vidsPL.end())
        LOG_IF(ERROR, "the Line id can not find");

    long lineStartPosition = distance(vidsPL.begin(), it);

    vidsPoint.resize(size_t(lineStartPosition-1));
    vidsLine.resize(vidsPL.size()-lineStartPosition);
    vidsPoint.assign(vidsPL.begin(), vidsPL.begin()+lineStartPosition);
    vidsLine.assign(vidsPL.begin()+lineStartPosition, vidsPL.end());

    for (auto &id : vidsPoint)
    {
        vPointResidues.push_back(ReprojectionError(problem, id).norm());
    }

    for (auto &id : vidsLine)
    {
        vLineResidues.push_back(ReprojectionError(problem, id).norm());
    }
}


void Optimizer::RemoveOutliers(ceres::Problem& problem, double threshold)
{
    std::vector<ceres::ResidualBlockId> ids;
    problem.GetResidualBlocks(&ids);

    int count = 0;
    for (auto & id: ids)
    {
        if (ReprojectionError(problem, id).norm() > threshold)
        {
            problem.RemoveResidualBlock(id);
            count++;
        }
    }

    if (ids.size() == 0)
    {
        LOG(ERROR) << "the number of residuals is zero " << endl;
        return;
    }

    LOG_IF(WARNING, (count/ids.size()) > 0.25) << " outliers is not a few: " << (count/ids.size()) ;
    LOG_IF(ERROR, (count/ids.size()) > 0.5) << " too much outliers: " << (count/ids.size()) << endl;
}

double Optimizer::VectorStdvMad(vector<double> vresidues_)
{
    if (vresidues_.empty())
        return 0.0;

    size_t num = vresidues_.size();

    vector<double> vresidues;
    vresidues.assign(vresidues_.begin(), vresidues_.end());

    sort(vresidues.begin(), vresidues.end());

    double median = vresidues[num/2];
    for (auto residual : vresidues)
        residual = fabs(residual - median);

    sort(vresidues.begin(), vresidues.end());

    double MAD = 1.4826*vresidues[num/2];

    return MAD;
}

void Optimizer::PoseOptimization(Frame *pFrame)
{
    Eigen::Matrix3d K;
    Eigen::Matrix2d pointSqrtInforMatrix;
    Eigen::Matrix2d lineSqrtInforMatrix;

    K = pFrame->mpCamera->GetCameraIntrinsic();
    size_t frameID = pFrame->GetFrameID();

    cv::Mat extrinsic(7, 1, CV_64FC1);

    {
        extrinsic.ptr<double>()[0] = pFrame->Tcw.unit_quaternion().x();
        extrinsic.ptr<double>()[1] = pFrame->Tcw.unit_quaternion().y();
        extrinsic.ptr<double>()[2] = pFrame->Tcw.unit_quaternion().z();
        extrinsic.ptr<double>()[3] = pFrame->Tcw.unit_quaternion().w();
        extrinsic.ptr<double>()[4] = pFrame->Tcw.translation()[0];
        extrinsic.ptr<double>()[5] = pFrame->Tcw.translation()[1];
        extrinsic.ptr<double>()[6] = pFrame->Tcw.translation()[2];
    }

    cout << pFrame->Tcw.unit_quaternion().coeffs() << endl;
    cout << pFrame->Tcw.translation() << endl;

    ceres::Problem problem;

    problem.AddParameterBlock(extrinsic.ptr<double>(), 7, new PoseLocalParameterization());

    ceres::LossFunction* lossfunction = new ceres::CauchyLoss(1);   // loss function make bundle adjustment robuster. HuberLoss

    // add the MapPoint parameterblocks and residuals
    for (auto pMapPoint : pFrame->mvpMapPoint)
    {
        if (pMapPoint->mPosew.isZero())
            continue;

        if (pFrame->GetFrameID() != 0)
            if (pMapPoint->GetObservedNum() <= 2)
                continue;

        if (!pMapPoint->mmpPointFeature2D[frameID]->mbinlier)
            continue;

        pointSqrtInforMatrix = Eigen::Matrix2d::Identity()*sqrt(pFrame->mvPointInvLevelSigma2[pMapPoint->mmpPointFeature2D[frameID]->mlevel]);

        Eigen::Vector2d observed = pMapPoint->mmpPointFeature2D[frameID]->mpixel;

        ceres::CostFunction *costfunction = new ReprojectionErrorSE3(K(0, 0), K(1, 1), K(0, 2), K(1, 2),
                                                                     observed[0], observed[1], pointSqrtInforMatrix);

        problem.AddResidualBlock(costfunction, lossfunction, extrinsic.ptr<double>(), &pMapPoint->mPosew.x());

        problem.AddParameterBlock(&pMapPoint->mPosew.x(), 3);
    }

    // add the MapLine parameterblocks and residuals
    for (auto pMapLine : pFrame->mvpMapLine)
    {
        if (pMapLine->mPoseStartw.isZero() || pMapLine->mPoseEndw.isZero())
            continue;

        if (pFrame->GetFrameID() != 0)
            if (pMapLine->GetObservedNum() <= 2)
                continue;

        if (!pMapLine->mmpLineFeature2D[frameID]->mbinlier)
            continue;

        lineSqrtInforMatrix = Eigen::Matrix2d::Identity()*sqrt(pFrame->mvLineInvLevelSigma2[pMapLine->mmpLineFeature2D[frameID]->mlevel]);

        Eigen::Vector2d observedStart;
        Eigen::Vector2d observedEnd;
        Eigen::Vector3d observedLineCoef;

        observedStart = pMapLine->mmpLineFeature2D[frameID]->mStartpixel;
        observedEnd = pMapLine->mmpLineFeature2D[frameID]->mEndpixel;
        observedLineCoef = pMapLine->mmpLineFeature2D[frameID]->mLineCoef;

        ceres::CostFunction *costFunction = new ReprojectionLineErrorSE3(K(0, 0), K(1, 1), K(0, 2), K(1, 2),
                                                                         observedStart, observedEnd, observedLineCoef, lineSqrtInforMatrix);

//        double cost;
//        cost = ComputeMapLineCost(pMapLine, pFrame->Tcw.unit_quaternion(), pFrame->Tcw.translation(), K, frameID);

        problem.AddResidualBlock(costFunction, lossfunction, extrinsic.ptr<double>(), &pMapLine->mPoseStartw.x(),
                                 &pMapLine->mPoseEndw.x());

        problem.AddParameterBlock(&pMapLine->mPoseStartw.x(), 3);
        problem.AddParameterBlock(&pMapLine->mPoseEndw.x(), 3);
    }

    RemoveOutliers(problem, 25);

//    vector<double> vresiduals;
//    vresiduals = GetReprojectionErrorNorms(problem);
//
//    for (auto residual : vresiduals)
//    {
//        cout << residual << endl;
//    }

    ceres::Solver::Options options;
//    options.num_threads = 4;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    options.max_solver_time_in_seconds = 0.1;

    cout << "pose optimization " << endl;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    {
        // it is very important that the scale data is the first position in the quaternion data type in the eigen
        // but if you use pointer to use the data, the scale date is the last position.
        pFrame->Tcw.setQuaternion(Eigen::Quaterniond(extrinsic.ptr<double>()[3], extrinsic.ptr<double>()[0],
                                                     extrinsic.ptr<double>()[1],extrinsic.ptr<double>()[2]));

        pFrame->Tcw.setQuaternion(pFrame->Tcw.unit_quaternion().normalized());
        pFrame->Tcw.translation()[0] = extrinsic.ptr<double>()[4];
        pFrame->Tcw.translation()[1] = extrinsic.ptr<double>()[5];
        pFrame->Tcw.translation()[2] = extrinsic.ptr<double>()[6];
    }

    cout << pFrame->Tcw.unit_quaternion().coeffs() << endl;
    cout << pFrame->Tcw.translation() << endl;

//    for (auto pMapLine : pFrame->mvpMapLine)
//    {
//        if (pMapLine->mPoseStartw.isZero() || pMapLine->mPoseEndw.isZero())
//            continue;
//        cout << pMapLine->mID << " : "
//             << pMapLine->mPoseStartw.transpose() << " | "
//             << pMapLine->mPoseEndw.transpose() << endl;
//    }

    if (!summary.IsSolutionUsable())
    {
        cout << "Pose Optimization failed." << endl;
    }
    else
    {
        // Display statistics about the minimization
        cout << summary.BriefReport() << endl
             << " residuals number: " << summary.num_residuals << endl
             << " Initial RMSE: " << sqrt(summary.initial_cost / summary.num_residuals) << endl
             << " Final RMSE: " << sqrt(summary.final_cost / summary.num_residuals) << endl
             << " Time (s): " << summary.total_time_in_seconds << endl;
    }

//    vresiduals = GetReprojectionErrorNorms(problem);
//
//    for (auto residual : vresiduals)
//    {
//        cout << residual << endl;
//    }

} // void Optimizer::PoseOptimization(Frame *pFrame)


void Optimizer::PnPResultOptimization(Frame *pFrame, Sophus::SE3d &PoseInc,
                                      vector<PointFeature2D *> &vpPointFeature2DLast,
                                      vector<PointFeature2D *> &vpPointFeature2DCur,
                                      vector<LineFeature2D *> &vpLineFeature2DLast,
                                      vector<LineFeature2D *> &vpLineFeature2DCur)
{
    vector<PointFeature2D *> vpPointFeature2DInliers;
    vector<LineFeature2D *> vpLineFeature2DInliers;

    Eigen::Matrix3d K;
    Eigen::Matrix2d pointInforMatrix;
    Eigen::Matrix2d pointSqrtInforMatrix;
    Eigen::Matrix2d lineInforMatrix;
    Eigen::Matrix2d lineSqrtInforMatrix;

    K = pFrame->mpCamera->GetCameraIntrinsic();

    cv::Mat extrinsic(7, 1, CV_64FC1);

    {
        extrinsic.ptr<double>()[0] = PoseInc.unit_quaternion().x();
        extrinsic.ptr<double>()[1] = PoseInc.unit_quaternion().y();
        extrinsic.ptr<double>()[2] = PoseInc.unit_quaternion().z();
        extrinsic.ptr<double>()[3] = PoseInc.unit_quaternion().w();
        extrinsic.ptr<double>()[4] = PoseInc.translation()[0];
        extrinsic.ptr<double>()[5] = PoseInc.translation()[1];
        extrinsic.ptr<double>()[6] = PoseInc.translation()[2];
    }

    ceres::Problem problem;

    problem.AddParameterBlock(extrinsic.ptr<double>(), 7, new PoseLocalParameterization());

    ceres::LossFunction* lossfunction = new ceres::CauchyLoss(1);   // loss function make bundle adjustment robuster. HuberLoss

    CHECK(vpPointFeature2DLast.size() == vpPointFeature2DCur.size());
    CHECK(vpLineFeature2DLast.size() == vpLineFeature2DCur.size());

    // add the MapPoint parameterblocks and residuals
    for (int i = 0; i < vpPointFeature2DLast.size(); i++)
    {
        if (!vpPointFeature2DLast[i]->mbinlier)
            continue;

        pointSqrtInforMatrix = Eigen::Matrix2d::Identity()*sqrt(pFrame->mvPointInvLevelSigma2[vpPointFeature2DCur[i]->mlevel]);


        // the outliers are considered in the current frame, not in the last frame
        vpPointFeature2DInliers.emplace_back(vpPointFeature2DCur[i]);

        ceres::CostFunction *costfunction = new ReprojectionErrorSE3(K(0, 0), K(1, 1), K(0, 2), K(1, 2),
                                                                     vpPointFeature2DCur[i]->mpixel[0], vpPointFeature2DCur[i]->mpixel[1],
                                                                     pointSqrtInforMatrix);

        problem.AddResidualBlock( costfunction, lossfunction, extrinsic.ptr<double>(), &vpPointFeature2DLast[i]->mPoint3dw.x());

        problem.AddParameterBlock(&vpPointFeature2DLast[i]->mPoint3dw.x(), 3);

        problem.SetParameterBlockConstant(&vpPointFeature2DLast[i]->mPoint3dw.x());
    }

    // add the MapLine parameterblocks and residuals
    for (size_t i = 0; i < vpLineFeature2DLast.size(); i++)
    {
        if (!vpLineFeature2DLast[i]->mbinlier)
            continue;

        lineSqrtInforMatrix = Eigen::Matrix2d::Identity()*sqrt(pFrame->mvLineInvLevelSigma2[vpLineFeature2DCur[i]->mlevel]);

        // the outliers are considered in the current frame, not in the last frame
        vpLineFeature2DInliers.emplace_back(vpLineFeature2DCur[i]);

        ceres::CostFunction *costFunction = new ReprojectionLineErrorSE3(K(0, 0), K(1, 1), K(0, 2), K(1, 2),
                                                                         vpLineFeature2DCur[i]->mStartpixel,
                                                                         vpLineFeature2DCur[i]->mEndpixel, lineSqrtInforMatrix);

        problem.AddResidualBlock(costFunction, lossfunction, extrinsic.ptr<double>(),
                                 &vpLineFeature2DLast[i]->mStartPoint3dw.x(),
                                 &vpLineFeature2DLast[i]->mEndPoint3dw.x());

        problem.SetParameterBlockConstant(&vpLineFeature2DLast[i]->mStartPoint3dw.x());
        problem.SetParameterBlockConstant(&vpLineFeature2DLast[i]->mEndPoint3dw.x());
    }

    ceres::Solver::Options options;
//    options.num_threads = 4;
    options.max_num_iterations = Config::maxIters();
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = false;
    options.max_solver_time_in_seconds = 0.1;

    cout << "PnP optimization " << endl;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    {
        // it is very important that the scale data is the first position in the quaternion data type in the eigen
        // but if you use pointer to use the data, the scale date is the last position.
        PoseInc.setQuaternion(Eigen::Quaterniond(extrinsic.ptr<double>()[3], extrinsic.ptr<double>()[0],
                                                 extrinsic.ptr<double>()[1],extrinsic.ptr<double>()[2]));

        PoseInc.setQuaternion(PoseInc.unit_quaternion().normalized());
        PoseInc.translation()[0] = extrinsic.ptr<double>()[4];
        PoseInc.translation()[1] = extrinsic.ptr<double>()[5];
        PoseInc.translation()[2] = extrinsic.ptr<double>()[6];
    }

    if (!summary.IsSolutionUsable())
    {
        cout << "PnP optimization failed." << endl;
    }
    else
    {
        // Display statistics about the minimization
//        cout << summary.BriefReport() << endl
//             << " residuals number: " << summary.num_residuals << endl
//             << " Initial RMSE: " << sqrt(summary.initial_cost / summary.num_residuals) << endl
//             << " Final RMSE: " << sqrt(summary.final_cost / summary.num_residuals) << endl
//             << " Time (s): " << summary.total_time_in_seconds << endl;
    }

//    vector<double> vresiduals;
//    vresiduals = GetReprojectionErrorNorms(problem);
//
//    for (auto residual : vresiduals)
//    {
//        cout << residual << endl;
//    }

    vector<double> vPointResiduals, vLineResiduals;
    GetPLReprojectionErrorNorms(problem, &vpPointFeature2DLast[0]->mPoint3dw.x(), &vpLineFeature2DLast[0]->mStartPoint3dw.x(),
                                vPointResiduals, vLineResiduals);

    double pointInlierth = Config::inlierK()*VectorStdvMad(vPointResiduals);
    double lineInlierth = Config::inlierK()*VectorStdvMad(vLineResiduals);

    CHECK(vPointResiduals.size() == vpPointFeature2DInliers.size());
    CHECK(vLineResiduals.size() == vpLineFeature2DInliers.size());

    for (size_t i = 0; i < vPointResiduals.size(); i++)
    {
        if (vPointResiduals[i] > pointInlierth)
            vpPointFeature2DInliers[i]->mbinlier = false;
    }

    for (size_t i = 0; i < vLineResiduals.size(); i++)
    {
        if (vLineResiduals[i] > lineInlierth)
            vpLineFeature2DInliers[i]->mbinlier = false;
    }

} // Sophus::SE3 Optimizer::PnPResultOptimization(Frame *pFrame, Sophus::SE3 PoseInc)

void Optimizer::LocalBundleAdjustment(KeyFrame *pKeyFrame, bool *pbStopFlag, Map *pMap)
{
    Eigen::Matrix3d K;
    list<KeyFrame*> lLocalKeyFrames;

    K = pKeyFrame->mpCamera->GetCameraIntrinsic();

    lLocalKeyFrames.push_back(pKeyFrame);
    pKeyFrame->mBALocalForKF = pKeyFrame->GetFrameID();

    const vector<KeyFrame*> vNeighborKFs = pKeyFrame->GetVectorCovisibleKeyFrames();

    for (auto pKFi : vNeighborKFs)
    {
        pKFi->mBALocalForKF = pKeyFrame->GetFrameID();

        if (!pKFi->isBad())
            lLocalKeyFrames.push_back(pKFi);
    }

    list<MapPoint*> lLocalMapPoints;
    list<MapLine*> lLocalMapLines;

    for (auto pKFi : lLocalKeyFrames)
    {
        vector<MapPoint*> vpMPs = pKFi->GetMapPointMatches();
        for (auto pMP : vpMPs)
        {
            // TODO: just for the test
            CHECK_NOTNULL(pMP);
            if (pMP)
            {
                if (!pMP->isBad())
                {
                    if (pMP->mBALocalForKF != pKeyFrame->GetFrameID())
                    {
                        lLocalMapPoints.push_back(pMP);
                        pMP->mBALocalForKF = pKeyFrame->GetFrameID();
                    }
                }
            }
        }

        vector<MapLine*> vpMLs = pKFi->GetMapLineMatches();
        for (auto pML : vpMLs)
        {
            CHECK_NOTNULL(pML);
            if (pML)
            {
                if (!pML->isBad())
                {
                    if (pML->mBALocalForKF != pKeyFrame->GetFrameID())
                    {
                        lLocalMapLines.push_back(pML);
                        pML->mBALocalForKF = pKeyFrame->GetFrameID();
                    }
                }
            }
        }
    } // for (auto pKFi : lLocalKeyFrames)

    set<KeyFrame*> lFixedCameras;
    for (auto pMP : lLocalMapPoints)
    {
        vector<Frame*> observations = pMP->GetObservedFrame();

        for (auto pFrame : observations)
        {
            if (!pFrame->isKeyFrame())
                continue;

            KeyFrame* pKFi = pFrame->mpKeyFrame;

            if (pKFi->mBALocalForKF != pKeyFrame->GetFrameID() && pKFi->mBAFixedForKF != pKeyFrame->GetFrameID())
            {
                pKFi->mBAFixedForKF = pKeyFrame->GetFrameID();
                if (!pKFi->isBad())
                    lFixedCameras.insert(pKFi);
            }
        }
    }

    for (auto pML : lLocalMapLines)
    {
        vector<Frame*> observations = pML->GetObservedFrame();

        for (auto pFrame : observations)
        {
            if (!pFrame->isKeyFrame())
                continue;

            KeyFrame* pKFi = pFrame->mpKeyFrame;

            if (pKFi->mBAFixedForKF != pKeyFrame->GetFrameID() && pKFi->mBAFixedForKF != pKeyFrame->GetFrameID())
            {
                pKFi->mBAFixedForKF = pKeyFrame->GetFrameID();
                if (!pKFi->isBad())
                    lFixedCameras.insert(pKFi);
            }
        }
    }

    ceres::Problem problem;
    ceres::LossFunction* lossfunction = new ceres::CauchyLoss(1);

    Eigen::Matrix2d pointSqrtInforMatrix;
    Eigen::Matrix2d lineSqrtInforMatrix;

    for (auto pKeyFrame : lLocalKeyFrames)
    {
        problem.AddParameterBlock(pKeyFrame->Tcw.data(), 7, new PoseLocalParameterization());
    }

//    for (auto pFixedKeyFrame : lFixedCameras)
//    {
//        problem.SetParameterBlockConstant(pFixedKeyFrame->Tcw.data());
//    }

    cout << "ComputeMapPointCost: " << endl;

    for (auto pMapPoint : lLocalMapPoints)
    {
        if (pMapPoint->isBad())
            continue;

        if (pMapPoint->GetPose().isZero())
            continue;

        if (pMapPoint->GetObservedNum() <= 2)
            continue;

        vector<Frame*> observationPoints = pMapPoint->GetObservedFrame();

        for (auto pFramei : observationPoints)
        {
            if (!pFramei->isKeyFrame())
                continue;

            KeyFrame *pKFi = pFramei->mpKeyFrame;

            PointFeature2D *pPointFeature2D = pMapPoint->mmpPointFeature2D[pKFi->GetFrameID()];

            // TODO there should to add the thread mutex
            if (!pPointFeature2D->mbinlier)
                continue;

            pointSqrtInforMatrix = Eigen::Matrix2d::Identity()*sqrt(pFramei->mvPointInvLevelSigma2[pPointFeature2D->mlevel]);

            // TODO thread mutex
            Eigen::Vector2d observed = pPointFeature2D->mpixel;

            ceres::CostFunction *costfunction = new ReprojectionErrorSE3(K(0, 0), K(1, 1), K(0, 2), K(1, 2),
                                                                         observed[0], observed[1], pointSqrtInforMatrix);

            pMapPoint->mPosew_ = pMapPoint->mPosew;

            problem.AddResidualBlock(costfunction, lossfunction, pKFi->Tcw.data(), &pMapPoint->mPosew_.x());

            problem.AddParameterBlock(&pMapPoint->mPosew_.x(), 3);

//            cout << Optimizer::ComputeMapPointCost(pMapPoint, pKFi->Tcw.unit_quaternion(), pKFi->Tcw.translation(),
//                                                   K, pKFi->GetFrameID()) << endl;
        }
    }

    cout << "ComputeMapLineCost: " << endl;

    for (auto pMapLine : lLocalMapLines)
    {
        if (pMapLine->isBad())
            continue;

        if (pMapLine->GetPoseStart().isZero() || pMapLine->GetPoseEnd().isZero())
            continue;

        if (pMapLine->GetObservedNum() <= 2)
            continue;

        vector<Frame*> observationLines = pMapLine->GetObservedFrame();

        for (auto pFramei : observationLines)
        {
            if (!pFramei->isKeyFrame())
                continue;

            KeyFrame *pKFi = pFramei->mpKeyFrame;

            LineFeature2D *pLineFeature2d = pMapLine->mmpLineFeature2D[pKFi->GetFrameID()];

            if (!pLineFeature2d->mbinlier)
                continue;

            lineSqrtInforMatrix = Eigen::Matrix2d::Identity()*sqrt(pFramei->mvLineInvLevelSigma2[pLineFeature2d->mlevel]);

            Eigen::Vector2d observedStart;
            Eigen::Vector2d observedEnd;
            Eigen::Vector3d observedLineCoef;

            observedStart = pLineFeature2d->mStartpixel;
            observedEnd = pLineFeature2d->mEndpixel;
            observedLineCoef = pLineFeature2d->mLineCoef;

            ceres::CostFunction *costFunction = new ReprojectionLineErrorSE3(K(0, 0), K(1, 1), K(0, 2), K(1, 2),
                                                                             observedStart, observedEnd, observedLineCoef, lineSqrtInforMatrix);

            pMapLine->mPoseEndw_ = pMapLine->mPoseEndw;
            pMapLine->mPoseStartw_ = pMapLine->mPoseStartw;

            problem.AddResidualBlock(costFunction, lossfunction, pKFi->Tcw.data(), &pMapLine->mPoseStartw_.x(),
                                     &pMapLine->mPoseEndw_.x());

            problem.AddParameterBlock(&pMapLine->mPoseStartw_.x(), 3);
            problem.AddParameterBlock(&pMapLine->mPoseEndw_.x(), 3);

            cout << ComputeMapLineCost(pMapLine, pKFi->Tcw.unit_quaternion(), pKFi->Tcw.translation(), K, pKFi->GetFrameID()) << endl;
        }
    }

    cout << "GetReprojectionErrorNorms: " << endl;

    vector<double> vresiduals;
    vresiduals = GetReprojectionErrorNorms(problem);

    for (auto residual : vresiduals)
    {
        cout << residual << endl;
    }

    RemoveOutliers(problem, 25);

    cout << pKeyFrame->Tcw.matrix3x4() << endl;

    ceres::Solver::Options options;
//    options.num_threads = 4;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    options.max_solver_time_in_seconds = 0.3;

    cout << "local Bundle Adjustment optimization " << endl;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    if (!summary.IsSolutionUsable())
    {
        cout << "Local Bundle Adjustment failed." << endl;
    }
    else
    {
        cout << summary.BriefReport() << endl
             << " residuals number: " << summary.num_residuals << endl
             << " Initial RMSE: " << sqrt(summary.initial_cost / summary.num_residuals) << endl
             << " Final RMSE: " << sqrt(summary.final_cost / summary.num_residuals) << endl
             << " Time (s): " << summary.total_time_in_seconds << endl;
    }

    for (auto pKeyFrame : lLocalKeyFrames)
    {
        pKeyFrame->Tcw.setQuaternion(pKeyFrame->Tcw.unit_quaternion().normalized());
    }

    cout << pKeyFrame->Tcw.matrix3x4() << endl;

} // void Optimizer::LocalBundleAdjustment(KeyFrame *pKeyFrame, bool *pbStopFlag, Map *pMap)

} // namespace PL_VO