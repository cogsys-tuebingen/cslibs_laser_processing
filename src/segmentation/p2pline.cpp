/// HEADER
#include <utils_laser_processing/segmentation/p2pline.h>

/// COMPONENT
#include <utils_laser_processing/segmentation/utils.hpp>

using namespace lib_laser_processing;

P2PLine::P2PLine(const double sigma, const double max_distance) :
    sigma_(sigma),
    max_distance_(max_distance)
{
}

P2PLine::~P2PLine()
{
}

void P2PLine::segmentation(const std::vector<float> &reading, const float angular_res, const float min_angle,
                                         const float min_rho, const float max_rho, std::vector<std::vector<LaserBeam> > &lines)
{
    /// PREPARATION
    lines.clear();
    std::vector<LaserBeam> points;
    utils::polarToCartesian(reading, angular_res, min_angle, min_rho, max_rho, points);

    /// COMPUTATION
    Eigen::ParametrizedLine<double,2> line;

    assert(reading.size() > 2);
    std::vector<LaserBeam>::iterator first    = points.begin();
    std::vector<LaserBeam>::iterator second   = first + 1;
    std::vector<LaserBeam>::iterator last_fit = first;
    std::vector<LaserBeam>::iterator end      = points.end();
    std::vector<LaserBeam> segment;

    segment.push_back(*first);
    while(true) {
        Eigen::Vector2d direction(second->pos - first->pos);
        direction.normalize();
        line = Eigen::ParametrizedLine<double,2>(first->pos, direction);

        if(second == end) {
            lines.push_back(segment);
            return;
        } else {
            double dist = utils::distance(last_fit, second);

            if(dist > max_distance_) {
                lines.push_back(segment);
                first       = second;
                last_fit    = first;
                /// BUFFER
                segment.clear();
                segment.push_back(*first);
            } else if(!utils::withinLineFit(first, second + 1, line, sigma_)) {
                lines.push_back(segment);
                first = last_fit;

                segment.clear();
                segment.push_back(*first);
            } else {
                /// CONTINUE
                segment.push_back(*second);
                last_fit = second;
            }
            ++second;
        }
    }
}
