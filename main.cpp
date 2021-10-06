#include <chrono>
#include <random>

#include "FMII.h"
// #include "matplotlibcpp.h"
// namespace plt = matplotlibcpp;

template <int Dim = 2>
void plot_line(const Line<Dim>& line) {
    VectorNd<Dim> start_p = line.start + line.t1 * line.dir;
    VectorNd<Dim> end_p = line.start + line.t2 * line.dir;
    
    for (int i = 0; i < Dim - 1; i++) {
        std::cout << start_p[i] << ',';
    }
    std::cout << start_p[Dim - 1] << '\n';

    for (int i = 0; i < Dim - 1; i++) {
        std::cout << end_p[i] << ',';
    }
    std::cout << end_p[Dim - 1] << '\n';
}

int main() {
    default_random_engine gen;
    gen.seed(chrono::system_clock::now().time_since_epoch().count());
    uniform_real_distribution<double> dist(-1.0, 1.0);

    // plt::figure_size(1440, 720);
    // generate some random image_lines

    {
        LineVector<2> image_lines;
        for (int i = 0; i < 5; i++) {
            image_lines.push_back({0.0, 4.0, {dist(gen) * 15, dist(gen) * 15}, {dist(gen), dist(gen)}});
            auto& line = image_lines.back();
            line.normalize();
            line.recalc_start();
            plot_line(line);
        }
        std::cout << "next\n";
        Eigen::Rotation2Dd rot(dist(gen) * 3.14);
        Eigen::Vector2d trans(6.0, 7.0);
        LineVector<2> model_lines = image_lines;
        for (auto& line : model_lines) {
            line.dir = rot * line.dir;
            line.start = rot * line.start + trans;
            line.t1 += dist(gen);
            line.t2 += dist(gen);
            plot_line(line);
        }
        std::cout << "next\n";

        vector<LinePair<2>> pairings(image_lines.size());
        for (int i = 0; i < image_lines.size(); i++) {
            pairings[i].model = &model_lines[i];
            pairings[i].data = &image_lines[i];
            pairings[i].sn = 0.0;
        }
        FMII(pairings);

        for (auto& line : image_lines) {
            plot_line(line);
        }
        std::cout << "next" << std::endl;
    }

    {
        LineVector<3> image_lines;
        for (int i = 0; i < 5; i++) {
            image_lines.push_back({0.0, 8.0, {dist(gen) * 15, dist(gen) * 15, dist(gen) * 15}, {dist(gen), dist(gen), dist(gen)}});
            auto& line = image_lines.back();
            line.normalize();
            line.recalc_start();
            plot_line(line);
        }
        std::cout << "next\n";

        // Eigen::AngleAxisd rollAngle(0.45, Eigen::Vector3d::UnitZ());
        // Eigen::AngleAxisd yawAngle(0.0, Eigen::Vector3d::UnitY());
        // Eigen::AngleAxisd pitchAngle(0.0, Eigen::Vector3d::UnitX());
        // Eigen::Quaterniond q = rollAngle * yawAngle * pitchAngle;
        Eigen::Matrix3d rot = Eigen::Matrix3d::Zero();
        rot(2, 2) = 1;
        rot.topLeftCorner<2, 2>() = Eigen::Rotation2Dd(dist(gen) * 3.14).matrix();
        Eigen::Vector3d trans(0.0, 0.0, 0.0);
        auto model_lines = image_lines;
        for (auto& line : model_lines) {
            line.dir = rot * line.dir;
            line.start = rot * line.start + trans;
            line.t1 += dist(gen);
            line.t2 += dist(gen);
            plot_line(line);
        }
        std::cout << "next\n";
        vector<LinePair<3>> pairings(image_lines.size());
        for (int i = 0; i < image_lines.size(); i++) {
            pairings[i].model = &model_lines[i];
            pairings[i].data = &image_lines[i];
            pairings[i].sn = 0.0;
        }
        FMII(pairings);

        for (auto& line : image_lines) {
            plot_line(line);
        }
        std::cout << "next" << std::endl;
    }

    // // plt::legend();
    // plt::show();
}