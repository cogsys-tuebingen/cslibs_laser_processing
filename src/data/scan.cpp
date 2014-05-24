/// HEADER
#include <utils_laser_processing/data/scan.h>

using namespace lib_laser_processing;

Scan::Scan()
    : valid(false)
{

}

Scan::Scan(const std::vector<float> &ranges, float angle_min, float angle_increment)
    : angle_min(angle_min), angle_max(angle_min + angle_increment * ranges.size()), angle_increment(angle_increment), valid(true)
{
    rays.resize(ranges.size());

    std::vector<LaserBeam>::iterator output = rays.begin();
    double angle = angle_min;
    for(std::vector<float>::const_iterator input = ranges.begin(); input != ranges.end(); ++input, ++output) {
        *output = LaserBeam(angle, *input);
        angle += angle_increment;
    }
}

void Scan::getRanges(std::vector<float> &out) const
{
    out.resize(rays.size());

    std::vector<float>::iterator output = out.begin();
    for(std::vector<LaserBeam>::const_iterator input = rays.begin(); input != rays.end(); ++input, ++output) {
        *output = input->range;
    }
}
