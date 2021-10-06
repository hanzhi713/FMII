#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <iostream>
#include <vector>

template <int Dim>
using MatrixNd = Eigen::Matrix<double, Dim, Dim>;

template <int Dim>
using VectorNd = Eigen::Matrix<double, Dim, 1>;

using namespace std;
template <int Dim = 2>
struct Line {
    double t1, t2;
    VectorNd<Dim> start, dir;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    inline double pl_distance(const VectorNd<Dim>& point, double& t) const {
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

    VectorNd<Dim> midpoint() const {
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

template <int Dim = 2>
struct LinePair {
    const Line<Dim>* model;
    Line<Dim>* data;
    double sn;
};

template <int Dim = 2>
using LineVector = vector<Line<Dim>, Eigen::aligned_allocator<Line<Dim>>>;

template <int Dim = 2>
void FMII(vector<LinePair<Dim>>& pairings) {
    static_assert(Dim == 2 || Dim == 3);
    MatrixNd<Dim> inv_rot;
    VectorNd<Dim> inv_trans;
    for (int i = 0; i < 500; i++) {
        VectorNd<Dim> abar = VectorNd<Dim>::Zero();
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
            ccov.topLeftCorner<Dim, Dim>() += Ln * (pair.model->midpoint() - abar) * (pair.data->start + pair.sn * pair.data->dir - xbar).transpose() + pow(Ln, 3) * pair.model->dir * pair.data->dir.transpose() / 12.0;
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

        // use topLeftCorner just to avoid compile error
        // or can be resolved with C++17 if constexpr
        if (Dim == 2) {
            inv_rot.template topLeftCorner<2, 2>() = -q.matrix().bottomRightCorner<2, 2>();
        } else {
            auto angles = q.matrix().eulerAngles(0, 1, 2);
            std::cerr << "----------\n";
            std::cerr << angles << std::endl;
            Eigen::AngleAxisd rollAngle(angles[0], Eigen::Vector3d::UnitX());
            Eigen::AngleAxisd yawAngle(angles[1], Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd pitchAngle(angles[2], Eigen::Vector3d::UnitZ());
            // inv_rot = -q.matrix().topLeftCorner<Dim, Dim>();
            inv_rot = (rollAngle * yawAngle * pitchAngle).matrix().topLeftCorner<Dim, Dim>();
        }

        // std::cerr << q.matrix().eulerAngles(0, 1, 2) << std::endl;

        inv_trans = abar - inv_rot * xbar;
        double diffs = 0.0;
        for (auto& pair : pairings) {
            double new_sn = (pair.model->midpoint() - inv_trans).transpose() * (inv_rot * pair.data->dir);
            diffs += abs(new_sn - pair.sn);
            pair.sn = new_sn;
        }
        if (diffs / pairings.size() <= 1e-4) {
            cerr << "converged at i = " << i << endl;
            goto converged;
        }
    }
    cerr << "warning: not converged" << endl;
converged:
    for (auto& pair : pairings) {
        auto& line = *pair.data;
        line.dir = inv_rot * line.dir;
        line.start = inv_rot * line.start + inv_trans;
        line.recalc_start();
    }
}