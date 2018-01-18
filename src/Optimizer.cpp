//
// Created by rain on 18-1-3.
//

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
    Sophus::SE3 se3delta = Sophus::SE3::exp(Eigen::Map<const Eigen::Matrix<double, 6, 1, Eigen::ColMajor>>(delta));

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

    Eigen::Vector3d p = quaterd * point + trans;

    residuals[0] = fx*p[0]/p[2] + cx - observedx;
    residuals[1] = fy*p[1]/p[2] + cy - observedy;

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

            CHECK(fabs(Jse3.maxCoeff()) < 1e8);
            CHECK(fabs(Jse3.minCoeff()) < 1e8);
        }
        if(jacobians[1] != nullptr) // the jacobian for the camera pose optimization
        {
            Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor> > Jpoint(jacobians[1]);
            Jpoint = jacobian * quaterd.toRotationMatrix();

            CHECK(fabs(Jpoint.maxCoeff()) < 1e8);
            CHECK(fabs(Jpoint.minCoeff()) < 1e8);
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

    Eigen::Vector3d Startpoint3dC;
    Eigen::Vector3d Endpoint3dC;
    Eigen::Vector2d Startpoint2d;
    Eigen::Vector2d Endpoint2d;
    Eigen::Vector2d err;

    // the MapLine in the camera coordinate
//    cout << parameters[1][0] << " " << parameters[1][1] << " " << parameters[1][2] << endl;
//    cout << parameters[2][0] << " " << parameters[2][1] << " " << parameters[2][2] << endl;
//
//    cout << "startpoint3d: " << endl << Startpoint3d << endl;
//    cout << "endpoint3d: " << endl << Endpoint3d << endl;

    Startpoint3dC = quaterd*Startpoint3d + trans;
    Endpoint3dC = quaterd*Endpoint3d + trans;

    Startpoint2d[0] = fx*Startpoint3dC[0]/Startpoint3dC[2] + cx;
    Startpoint2d[1] = fy*Startpoint3dC[1]/Startpoint3dC[2] + cy;

    Endpoint2d[0] = fx*Endpoint3dC[0]/Endpoint3dC[2] + cx;
    Endpoint2d[1] = fy*Endpoint3dC[1]/Endpoint3dC[2] + cy;

    err[0] = lineCoef[0]*Startpoint2d[0] + lineCoef[1]*Startpoint2d[1] + lineCoef[2];
    err[1] = lineCoef[0]*Endpoint2d[0] + lineCoef[1]*Endpoint2d[1] + lineCoef[2];

//    err[0] = err[0]/err.norm();
//    err[1] = err[1]/err.norm();

    residuals[0] = err[0];
    residuals[1] = err[1];

    Eigen::Matrix<double, 2, 3, Eigen::RowMajor> jacobianStart;
    Eigen::Matrix<double, 2, 3, Eigen::RowMajor> jacobianEnd;
    Eigen::Matrix<double, 2, 3, Eigen::RowMajor> jacobian;

    jacobianStart << fx/Startpoint3dC[2],  0, -fx*Startpoint3dC[0]/Startpoint3dC[2]/Startpoint3dC[2],
                      0, fy/Startpoint3dC[2], -fy*Startpoint3dC[1]/Startpoint3dC[2]/Startpoint3dC[2];

    jacobianEnd << fx/Endpoint3dC[2],  0, -fx*Endpoint3dC[0]/Endpoint3dC[2]/Endpoint3dC[2],
                    0, fy/Endpoint3dC[2], -fy*Endpoint3dC[1]/Endpoint3dC[2]/Endpoint3dC[2];

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

            CHECK(fabs(Jse3.maxCoeff()) < 1e8);
            CHECK(fabs(Jse3.minCoeff()) < 1e8);
        }

        if(jacobians[1] != nullptr) // the jacobian for the camera pose optimization
        {
            Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor> > Jpoint(jacobians[1]);
            Jpoint.setZero();

            Jpoint.block<1, 3>(0, 0) = jacobian.block<1, 3>(0 ,0)*quaterd.toRotationMatrix();

            CHECK(fabs(Jpoint.maxCoeff()) < 1e8);
            CHECK(fabs(Jpoint.minCoeff()) < 1e8);
        }

        if (jacobians[2] != nullptr)
        {
            Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor> > Jpoint(jacobians[2]);
            Jpoint.setZero();

            Jpoint.block<1, 3>(1, 0) = jacobian.block<1, 3>(1 ,0)*quaterd.toRotationMatrix();

            CHECK(fabs(Jpoint.maxCoeff()) < 1e8);
            CHECK(fabs(Jpoint.minCoeff()) < 1e8);
        }

    }

    return true;

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

    problem.GetResidualBlocksForParameterBlock(pPointParameter, &vidsPoint);
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

    cout << pFrame->Tcw << endl;

    ceres::Problem problem;

    problem.AddParameterBlock(extrinsic.ptr<double>(), 7, new PoseLocalParameterization());

    ceres::LossFunction* lossfunction = new ceres::CauchyLoss(1);   // loss function make bundle adjustment robuster. HuberLoss

    // add the MapPoint parameterblocks and residuals
    for (int i = 0; i < pFrame->mvpMapPoint.size(); i++)
    {
        if (pFrame->mvpMapPoint[i]->mPosew.isZero())
            continue;

        Eigen::Vector2d observed = pFrame->mvpMapPoint[i]->mmpPointFeature2D[frameID]->mpixel;

        ceres::CostFunction *costfunction = new ReprojectionErrorSE3(K(0, 0), K(1, 1), K(0, 2), K(1, 2),
                                                                     observed[0], observed[1]);

        problem.AddResidualBlock(
                costfunction, lossfunction,
                extrinsic.ptr<double>(), &pFrame->mvpMapPoint[i]->mPosew.x());

        problem.AddParameterBlock(&pFrame->mvpMapPoint[i]->mPosew.x(), 3);
    }

    // add the MapLine parameterblocks and residuals
    for (auto pMapLine : pFrame->mvpMapLine)
    {

        if (pMapLine->mPoseStartw.isZero() || pMapLine->mPoseEndw.isZero())
            continue;

        Eigen::Vector2d observedStart;
        Eigen::Vector2d observedEnd;
        Eigen::Vector3d observedLineCoef;

        observedStart = pMapLine->mmpLineFeature2D[frameID]->mStartpixel;
        observedEnd = pMapLine->mmpLineFeature2D[frameID]->mEndpixel;
        observedLineCoef = pMapLine->mmpLineFeature2D[frameID]->mLineCoef;

        ceres::CostFunction *costFunction = new ReprojectionLineErrorSE3(K(0, 0), K(1, 1), K(0, 2), K(1, 2),
                                                                         observedStart, observedEnd, observedLineCoef);

        problem.AddResidualBlock(costFunction, lossfunction, extrinsic.ptr<double>(),
                                 &pMapLine->mPoseStartw.x(),
                                 &pMapLine->mPoseEndw.x());

        problem.AddParameterBlock(&pMapLine->mPoseStartw.x(), 3);
        problem.AddParameterBlock(&pMapLine->mPoseEndw.x(), 3);

//        cout << pMapLine->mID << " : "
//             << pMapLine->mPoseStartw.transpose() << " | "
//             << pMapLine->mPoseEndw.transpose() << endl;
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
    options.num_threads = 4;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    options.max_solver_time_in_seconds = 0.1;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    {
        // it is very important that the scale data is the first position in the quaternion data type in the eigen
        // but if you use pointer to use the data, the scale date is the last position.
        pFrame->Tcw.setQuaternion(Eigen::Quaterniond(extrinsic.ptr<double>()[3], extrinsic.ptr<double>()[0],
                                                     extrinsic.ptr<double>()[1],extrinsic.ptr<double>()[2]));

        pFrame->Tcw.so3().unit_quaternion().norm();
        pFrame->Tcw.translation()[0] = extrinsic.ptr<double>()[4];
        pFrame->Tcw.translation()[1] = extrinsic.ptr<double>()[5];
        pFrame->Tcw.translation()[2] = extrinsic.ptr<double>()[6];
    }

    cout << pFrame->Tcw << endl;

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
        cout << "Bundle Adjustment failed." << endl;
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


void Optimizer::PnPResultOptimization(Frame *pFrame, Sophus::SE3 &PoseInc, vector<cv::Point3d> &vPoint3d,
                                      const vector<cv::Point2d> &vPoint2d, vector<LineFeature2D*> &vpLineFeature2D,
                                      const vector<cv::Point2d> &vLineStart2d, const vector<cv::Point2d> &vLineEnd2d)
{
    Eigen::Matrix3d K;
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

    // add the MapPoint parameterblocks and residuals
    for (int i = 0; i < vPoint3d.size(); i++)
    {
        ceres::CostFunction *costfunction = new ReprojectionErrorSE3(K(0, 0), K(1, 1), K(0, 2), K(1, 2),
                                                                     vPoint2d[i].x, vPoint2d[i].y);

        problem.AddResidualBlock( costfunction, lossfunction, extrinsic.ptr<double>(), &vPoint3d[i].x);

        problem.AddParameterBlock(&vPoint3d[i].x, 3);
    }

    // add the MapLine parameterblocks and residuals
    for (size_t i = 0; i < vpLineFeature2D.size(); i++)
    {

        ceres::CostFunction *costFunction = new ReprojectionLineErrorSE3(K(0, 0), K(1, 1), K(0, 2), K(1, 2),
                                                                         Converter::toVector2d(vLineStart2d[i]),
                                                                         Converter::toVector2d(vLineEnd2d[i]));

        problem.AddResidualBlock(costFunction, lossfunction, extrinsic.ptr<double>(),
                                 &vpLineFeature2D[i]->mStartPoint3dw.x(),
                                 &vpLineFeature2D[i]->mEndPoint3dw.x());

        problem.SetParameterBlockConstant(&vpLineFeature2D[i]->mStartPoint3dw.x());
        problem.SetParameterBlockConstant(&vpLineFeature2D[i]->mEndPoint3dw.x());
    }

//    RemoveOutliers(problem, 25);

    ceres::Solver::Options options;
    options.num_threads = 4;
    options.max_num_iterations = Config::maxIters();
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    options.max_solver_time_in_seconds = 0.1;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    {
        // it is very important that the scale data is the first position in the quaternion data type in the eigen
        // but if you use pointer to use the data, the scale date is the last position.
        PoseInc.setQuaternion(Eigen::Quaterniond(extrinsic.ptr<double>()[3], extrinsic.ptr<double>()[0],
                                                 extrinsic.ptr<double>()[1],extrinsic.ptr<double>()[2]));

        PoseInc.so3().unit_quaternion().norm();
        PoseInc.translation()[0] = extrinsic.ptr<double>()[4];
        PoseInc.translation()[1] = extrinsic.ptr<double>()[5];
        PoseInc.translation()[2] = extrinsic.ptr<double>()[6];
    }

    if (!summary.IsSolutionUsable())
    {
        cout << "Bundle Adjustment failed." << endl;
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

    vector<double> vPointResiduals, vLineResiduals;
    GetPLReprojectionErrorNorms(problem, &vPoint3d[0].x, &vpLineFeature2D[0]->mStartPoint3dw.x(),
                                vPointResiduals, vLineResiduals);

    double pointInlierth = Config::inlierK()*VectorStdvMad(vPointResiduals);
    double lineInlierth = Config::inlierK()*VectorStdvMad(vLineResiduals);

    for (auto pointResidual : vPointResiduals)
    {
        if (pointResidual > pointInlierth)
            pFrame->mvPnPPointOutliers.push_back(true);
        else
            pFrame->mvPnPPointOutliers.push_back(false);
    }

    for (auto lineResidual : vLineResiduals)
    {
        if (lineResidual > lineInlierth)
            pFrame->mvPnPLineOutliers.push_back(true);
        else
            pFrame->mvPnPLineOutliers.push_back(false);
    }

    CHECK(pFrame->mvPnPPointOutliers.size() == vPoint3d.size());
    CHECK(pFrame->mvPnPLineOutliers.size() == vpLineFeature2D.size());

} // Sophus::SE3 Optimizer::PnPResultOptimization(Frame *pFrame, Sophus::SE3 PoseInc)

} // namespace PL_VO