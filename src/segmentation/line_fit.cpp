/// HEADER
#include <utils_laser_processing/segmentation/line_fit.h>

/// COMPONENT
#include <utils_laser_processing/segmentation/utils.hpp>

using namespace lib_laser_processing;

LineFit::LineFit(const double sigma, const double max_distance) :
    sigma_(sigma),
    max_distance_(max_distance)
{
}

LineFit::~LineFit()
{
}

void LineFit::segmentation(const Scan& scan, std::vector<Segment> &segments)
{
    segments.clear();

    Eigen::ParametrizedLine<double,2> line;
    Eigen::ParametrizedLine<double,2> last_line;

    assert(scan.rays.size() > 2);
    std::vector<LaserBeam>::const_iterator first    = scan.rays.begin();
    std::vector<LaserBeam>::const_iterator second   = first + 1;
    std::vector<LaserBeam>::const_iterator last_fit = first;
    std::vector<LaserBeam>::const_iterator end      = scan.rays.end();;
    while(true) {
        utils::regression2D(first, second + 1, line);
        if(second == end) {
            pushbackLineSegment(last_line, segments);
            return;
        } else {
            double dist = utils::distance(last_fit, second);
            if(dist > max_distance_) {
                pushbackLineSegment(last_line, segments);
                first       = second;
                last_fit    = first;
            } else if(!utils::withinLineFit(first, second  + 1, line, sigma_)) {
                pushbackLineSegment(last_line, segments);
                first = last_fit;
            } else {
                last_fit = second;
                last_line = line;
            }
            ++second;
        }
    }
}

void LineFit::pushbackLineSegment(Eigen::ParametrizedLine<double, 2> line, std::vector<Segment> &segments)
{
    segments.push_back(Segment());
    segments.back().rays.push_back(line.origin());
    segments.back().rays.push_back(LaserBeam(line.origin() + line.direction()));
}
