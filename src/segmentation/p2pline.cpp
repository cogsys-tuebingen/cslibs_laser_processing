/// HEADER
#include <cslibs_laser_processing/segmentation/p2pline.h>

/// COMPONENT
#include <cslibs_laser_processing/segmentation/utils.hpp>

using namespace lib_laser_processing;

P2PLine::P2PLine(const double sigma, const double max_distance) :
    sigma_(sigma),
    max_distance_(max_distance)
{
}

P2PLine::~P2PLine()
{
}

void P2PLine::segmentation(const Scan& scan, std::vector<Segment> &segments)
{
    /// PREPARATION
    segments.clear();

    /// COMPUTATION
    Eigen::ParametrizedLine<double,2> line;

    assert(scan.rays.size() > 2);
    std::vector<LaserBeam>::const_iterator first    = scan.rays.begin();
    std::vector<LaserBeam>::const_iterator second   = first + 1;
    std::vector<LaserBeam>::const_iterator last_fit = first;
    std::vector<LaserBeam>::const_iterator end      = scan.rays.end();
    Segment segment;

    //TODO: Add segment index support!

    segment.rays.push_back(*first);
    while(true) {
        Eigen::Vector2d direction(second->posX() - first->posX(), second->posY() - first->posY());
        direction.normalize();
        line = Eigen::ParametrizedLine<double,2>(Eigen::Vector2d(first->posX(), first->posY()), direction);

        if(second == end) {
            segments.push_back(segment);
            return;
        } else {
            if(second->valid()) {
                double dist = utils::distance(last_fit, second);
                if(dist > max_distance_) {
                    segments.push_back(segment);
                    first       = second;
                    last_fit    = first;
                    /// BUFFER
                    segment.rays.clear();
                    segment.rays.push_back(*first);
                } else if(!utils::withinLineFit(first, second + 1, line, sigma_)) {
                    segments.push_back(segment);
                    first = last_fit;

                    segment.rays.clear();
                    segment.rays.push_back(*first);
                } else {
                    /// CONTINUE
                    segment.rays.push_back(*second);
                    last_fit = second;
                }
            }
            ++second;
        }
    }
}
