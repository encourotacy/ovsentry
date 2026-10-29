#ifndef TOOLS__TRAJECTORY_HPP
#define TOOLS__TRAJECTORY_HPP

namespace tools {
struct Trajectory {
    bool unsolvable;
    double fly_time;
    double pitch;

    Trajectory(double v0, double d, double h, double k = 1e-3, double g = 9.6333);
};

}  // namespace tools

#endif  // TOOLS__TRAJECTORY_HPP
