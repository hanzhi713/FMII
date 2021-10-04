#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <iostream>
#include <vector>

using namespace std;
struct Line {
    double t1, t2;
    Eigen::Vector2d start, dir;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    inline double pl_distance(const Eigen::Vector2d& point, double& t) const {
        auto dvec = point - start;
        t = dvec.dot(dir);
        return (dvec - t * dir).norm();
    }

    void normalize() {
        dir /= dir.norm();
    }

    double length() const {
        return t2 - t1;
    }

    Eigen::Vector2d midpoint() const {
        return ((t1 + t2) * 0.5) * dir + start;
    }

    // recalculate the start point of the line so it is the point closest to the origin
    // a requirement of the FMII algorithm
    void recalc_start() {
        double t_start = start.dot(dir);
        t1 += t_start;
        t2 += t_start;
        start -= t_start * dir;
    }
};

struct LinePair {
    const Line *model;
    Line* data;
    double sn;
};

using LineVector = vector<Line, Eigen::aligned_allocator<Line>>;

bool FMII(vector<LinePair>& pairings) {
    Eigen::Rotation2Dd inv_rot;
    Eigen::Vector2d inv_trans;
    bool converged = false;
    for (int i = 0; i < 500; i++) {
        Eigen::Vector2d abar = Eigen::Vector2d::Zero();
        auto xbar = abar;

        double W = 0.0;
        for (auto& pair : pairings) {
            double Ln = pair.model->length();
            abar += Ln * pair.model->midpoint();
            xbar += Ln * (pair.data->start + pair.sn * pair.data->dir);
            W += Ln;
        }

        abar /= W;
        xbar /= W;

        Eigen::Matrix3d ccov = Eigen::Matrix3d::Zero();
        for (auto& pair : pairings) {
            double Ln = pair.model->length();
            ccov.block<2, 2>(0, 0) += Ln * (pair.model->midpoint() - abar) * (pair.data->start + pair.sn * pair.data->dir - xbar).transpose() + pow(Ln, 3) * pair.model->dir * pair.data->dir.transpose() / 12.0;
        }

        Eigen::Matrix3d Aij = ccov - ccov.transpose();
        Eigen::Vector3d Delta(Aij(1, 2), Aij(2, 0), Aij(0, 1));
        Eigen::Matrix4d Q;
        Q(0, 0) = ccov.trace();
        Q.block<1, 3>(0, 1) = Delta.transpose();
        Q.block<3, 1>(1, 0) = Delta;
        Q.block<3, 3>(1, 1) = ccov + ccov.transpose() - Q(0, 0) * Eigen::Matrix3d::Identity();

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> eigensolver(Q);
        Eigen::Quaterniond q;
        q.coeffs() = eigensolver.eigenvectors().col(3);
        auto angles = q.toRotationMatrix().eulerAngles(0, 1, 2);
        inv_rot = Eigen::Rotation2Dd(angles[0]);
        inv_trans = abar - inv_rot * xbar;
        double diffs = 0.0;
        for (auto& pair : pairings) {
            double new_sn = (pair.model->midpoint() - inv_trans).transpose() * (inv_rot * pair.data->dir);
            diffs += abs(new_sn - pair.sn);
            pair.sn = new_sn;
        }
        if (diffs / pairings.size() <= 1e-2) {
            cout << "converged at i = " << i << endl;
            converged = true;
            break;
        }
    }
    for (auto& pair : pairings) {
        auto& line = *pair.data;
        line.dir = inv_rot * line.dir;
        line.start = inv_rot * line.start + inv_trans;
        line.recalc_start();
    }
    if (!converged)
        cerr << "warning: not converged" << endl;
    return inv_rot.angle() < 0.05 && inv_trans.norm() < 0.01;
}