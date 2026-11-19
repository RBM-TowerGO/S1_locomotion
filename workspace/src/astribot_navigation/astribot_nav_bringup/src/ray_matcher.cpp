#include <algorithm>
#include <cmath>
#include <cstdint>

// Count the same half-cell samples as the NumPy matcher, up to the endpoint margin.
extern "C" void towergo_ray_fractions(
    const double* points, const double* lengths, int ray_count,
    const double* steps, int step_count, const double* laser, const double* pose,
    const double* origin, const double* rotation, double resolution,
    int width, int height, const uint8_t* free_cells, const uint8_t* occupied_cells,
    double* fractions)
{
    int valid = 0, known = 0, occupied = 0, rays_with_hit = 0;
    const double c = std::cos(pose[2]), s = std::sin(pose[2]);
    for (int i = 0; i < ray_count; ++i) {
        const double length = lengths[i];
        const double dx = (points[2*i] - laser[0]) / length;
        const double dy = (points[2*i+1] - laser[1]) / length;
        bool hit = false;
        for (int j = 0; j < step_count && steps[j] < length - 0.15; ++j) {
            ++valid;
            const double x = laser[0] + dx * steps[j], y = laser[1] + dy * steps[j];
            const double wx = x*c - y*s + pose[0] - origin[0];
            const double wy = x*s + y*c + pose[1] - origin[1];
            const double col = std::floor((wx*rotation[0] + wy*rotation[2]) / resolution);
            const double row = std::floor((wx*rotation[1] + wy*rotation[3]) / resolution);
            if (!(col >= 0 && col < width && row >= 0 && row < height)) continue;
            const auto index = static_cast<int>(row)*width + static_cast<int>(col);
            known += occupied_cells[index] || free_cells[index];
            if (occupied_cells[index]) {
                ++occupied;
                hit = true;
            }
        }
        rays_with_hit += hit;
    }
    fractions[0] = static_cast<double>(rays_with_hit) / ray_count;
    fractions[1] = static_cast<double>(known) / std::max(1, valid);
    fractions[2] = static_cast<double>(occupied) / std::max(1, valid);
}

