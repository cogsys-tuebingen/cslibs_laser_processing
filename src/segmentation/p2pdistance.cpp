#include <utils_laser_processing/segmentation/p2pdistance.h>
#include <utils_laser_processing/segmentation/utils.hpp>

using namespace lib_laser_processing;

P2PDistance::P2PDistance(const double max_distance) :
    max_distance_(max_distance)
{
}

P2PDistance::~P2PDistance()
{
}

void P2PDistance::segmentation(const std::vector<float> &reading, const float angular_res, const float min_angle,
                                           const float min_rho, const float max_rho, std::vector<std::vector<LaserBeam> > &segments)
{
    /// PREPARATION
    segments.clear();
    std::vector<LaserBeam> points;
    utils::polarToCartesian(reading, angular_res, min_angle, min_rho, max_rho, points);

    /// COMPUTATION
    if(points.size() > 1) {
        std::vector<LaserBeam>::iterator last = points.begin();
        std::vector<LaserBeam>::iterator curr = last + 1;
        std::vector<LaserBeam>  buffer;
        buffer.push_back(*last);
        for( ; curr != points.end() ; ++curr) {
            if(utils::distance(last, curr) > max_distance_) {
                segments.push_back(buffer);
                buffer.clear();
            }
            buffer.push_back(*curr);
            last = curr;
        }

        if(buffer.size() > 0) {
            segments.push_back(buffer);
        }
    }
}
