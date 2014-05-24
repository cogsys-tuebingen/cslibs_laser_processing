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

void LineFit::segmentation(const std::vector<float> &reading, const float angular_res, const float min_angle,
                           const float min_rho, const float max_rho, std::vector<std::vector<LaserBeam> > &segments)
{
    std::vector<LaserBeam> points;
    utils::polarToCartesian(reading, angular_res, min_angle, min_rho, max_rho, points);

    segments.clear();

    Eigen::ParametrizedLine<double,2> line;
    Eigen::ParametrizedLine<double,2> last_line;

    assert(reading.size() > 2);
    std::vector<LaserBeam>::iterator first    = points.begin();
    std::vector<LaserBeam>::iterator second   = first + 1;
    std::vector<LaserBeam>::iterator last_fit = first;
    std::vector<LaserBeam>::iterator end      = points.end();;
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

void LineFit::pushbackLineSegment(Eigen::ParametrizedLine<double, 2> line, std::vector<std::vector<LaserBeam> > &segments)
{
    segments.push_back(std::vector<LaserBeam>());
    segments.back().push_back(line.origin());
    segments.back().push_back(LaserBeam(line.origin() + line.direction()));
}
