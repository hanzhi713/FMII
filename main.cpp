#include <chrono>
#include <random>

#include "FMII.h"
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

template <int Dim=2>
void plot_line(const Line<Dim>& line, const string& name, const string& key) {
    Eigen::Vector2d start_p = line.start + line.t1 * line.dir;
    Eigen::Vector2d end_p = line.start + line.t2 * line.dir;
    // cout << start_p << "|" << end_p << endl;
    if (Dim == 2)
        plt::named_plot<double, double>(name, {start_p[0], end_p[0]}, {start_p[1], end_p[1]}, key);
    else
        plt::plot3<double>({start_p[0], end_p[0]}, {start_p[1], end_p[1]}, {start_p[2], end_p[2]});
    
}

int main() {
    default_random_engine gen;
    gen.seed(chrono::system_clock::now().time_since_epoch().count());
    uniform_real_distribution<double> dist(0.0, 1.0);

    plt::figure_size(1440, 720);
    plt::subplot(1, 2, 1);
    // generate some random image_lines

    {
        LineVector<2> image_lines;
        for (int i = 0; i < 5; i++) {
            image_lines.push_back({0.0, 3.0, {dist(gen) * 15, dist(gen) * 15}, {dist(gen), dist(gen)}});
            auto& line = image_lines.back();
            line.normalize();
            line.recalc_start();
            plot_line(line, "image", "b-");
        }
        Eigen::Rotation2Dd rot(0.45);
        Eigen::Vector2d trans(6.0, 7.0);
        LineVector<2> model_lines = image_lines;
        for (auto& line : model_lines) {
            line.dir = rot * line.dir;
            line.start = rot * line.start;
            line.start += trans;
            line.t1 -= dist(gen) * 2;
            line.t2 += dist(gen) * 2;
            plot_line(line, "model", "g-");
        }

        vector<LinePair<2>> pairings(image_lines.size());
        for (int i = 0; i < image_lines.size(); i++) {
            pairings[i].model = &model_lines[i];
            pairings[i].data = &image_lines[i];
            pairings[i].sn = 0.0;
        }
        FMII(pairings);

        for (auto& line : image_lines) {
            plot_line(line, "image on model", "y-");
        }
    }

    plt::subplot(1, 2, 2);
    {
        LineVector<3> image_lines;
        for (int i = 0; i < 5; i++) {
            image_lines.push_back({0.0, 3.0, {dist(gen) * 15, dist(gen) * 15, dist(gen) * 15}, {dist(gen), dist(gen), dist(gen)}});
            auto& line = image_lines.back();
            line.normalize();
            line.recalc_start();
            plot_line(line, "image", "b-");
        }
        Eigen::AngleAxisd rollAngle(0.45, Eigen::Vector3d::UnitZ());
        Eigen::AngleAxisd yawAngle(0.55, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd pitchAngle(0.65, Eigen::Vector3d::UnitX());
        Eigen::Quaterniond q = rollAngle * yawAngle * pitchAngle;
        Eigen::Matrix3d rot = q.matrix();
        Eigen::Vector3d trans(6.0, 7.0, 8.0);
        LineVector<3> model_lines = image_lines;
        for (auto& line : model_lines) {
            line.dir = rot * line.dir;
            line.start = rot * line.start;
            line.start += trans;
            line.t1 -= dist(gen) * 2;
            line.t2 += dist(gen) * 2;
            plot_line(line, "model", "g-");
        }

        vector<LinePair<3>> pairings(image_lines.size());
        for (int i = 0; i < image_lines.size(); i++) {
            pairings[i].model = &model_lines[i];
            pairings[i].data = &image_lines[i];
            pairings[i].sn = 0.0;
        }
        FMII(pairings);

        for (auto& line : image_lines) {
            plot_line(line, "image on model", "y-");
        }
    }

    plt::legend();
    plt::show();
}