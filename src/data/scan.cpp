/// HEADER
#include <cslibs_laser_processing/data/scan.h>

using namespace lib_laser_processing;

Scan::Scan()
    : valid(false)
{

}

Scan::Scan(const std::vector<float> &ranges, float angle_min, float angle_increment, float min_range, float max_range)
    : angle_min(angle_min), angle_max(angle_min + angle_increment * ranges.size()), angle_increment(angle_increment),
      range_min(min_range), range_max(max_range), valid(true)
{
    rays.resize(ranges.size());

    std::vector<LaserBeam>::iterator output = rays.begin();
    double angle = angle_min;
    for(std::vector<float>::const_iterator range = ranges.begin(); range != ranges.end(); ++range, ++output) {
        double r = *range;
        if(r > range_min && r < range_max) {
            *output = LaserBeam(angle, *range);
        } else {
            *output = LaserBeam();
        }
        angle += angle_increment;
    }
}

void Scan::getRanges(std::vector<float> &out) const
{
    out.resize(rays.size());

    std::vector<float>::iterator output = out.begin();
    for(std::vector<LaserBeam>::const_iterator input = rays.begin(); input != rays.end(); ++input, ++output) {
        *output = input->range();
    }
}
