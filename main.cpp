#include <chrono>
#include <random>

#include "FMII.h"
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;


void plot_line(const Line& line, const string& name, const string& key) {
    Eigen::Vector2d start_p = line.start + line.t1 * line.dir;
    Eigen::Vector2d end_p = line.start + line.t2 * line.dir;
    // cout << start_p << "|" << end_p << endl;
    plt::named_plot<double, double>(name, {start_p[0], end_p[0]}, {start_p[1], end_p[1]}, key);
}

int main() {
    default_random_engine gen;
    gen.seed(chrono::system_clock::now().time_since_epoch().count());
    uniform_real_distribution<double> dist(0.0, 1.0);

    plt::figure_size(1280, 720);
    // generate some random image_lines
    LineVector image_lines;
    for (int i = 0; i < 5; i++) {
        image_lines.push_back({0.0, 3.0, {dist(gen) * 15, dist(gen) * 15}, {dist(gen), dist(gen)}});
        auto& line = image_lines.back();
        line.normalize();
        line.recalc_start();
        plot_line(line, "image", "b-");
    }
    Eigen::Rotation2Dd rot(0.45);
    Eigen::Vector2d trans(6.0, 7.0);
    LineVector model_lines = image_lines;
    for (auto& line : model_lines) {
        line.dir = rot * line.dir;
        line.start = rot * line.start;
        line.start += trans;
        line.t1 -= dist(gen) * 2;
        line.t2 += dist(gen) * 2;
        plot_line(line, "model", "g-");
    }

    vector<LinePair> pairings(image_lines.size());
    for (int i = 0; i < image_lines.size(); i++) {
        pairings[i].model = &model_lines[i];
        pairings[i].data = &image_lines[i];
        pairings[i].sn = 0.0;
    }
    FMII(pairings);

    for (auto& line : image_lines) {
        plot_line(line, "image on model", "y-");
    }

    plt::legend();
    plt::show();
}