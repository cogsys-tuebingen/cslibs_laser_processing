#include <cslibs_laser_processing/segmentation/p2pdistance.h>
#include <cslibs_laser_processing/segmentation/utils.hpp>

using namespace lib_laser_processing;

P2PDistance::P2PDistance(const double max_distance) :
    max_distance_(max_distance)
{
}

P2PDistance::~P2PDistance()
{
}

void P2PDistance::segmentation(const Scan& scan, std::vector<Segment> &segments)
{
    /// PREPARATION
    segments.clear();

    /// COMPUTATION
    if(scan.rays.size() > 1) {
        std::vector<LaserBeam>::const_iterator last = scan.rays.begin();
        std::vector<LaserBeam>::const_iterator curr = last + 1;
        Segment buffer;
        buffer.rays.push_back(*last);
        buffer.start_idx = 0;

        for(std::size_t i = 1 ; i < scan.rays.size() ; ++i, ++curr) {
            if(curr->invalid()) {
                continue;
            }
            if(utils::distance(last, curr) > max_distance_) {
                segments.push_back(buffer);
                buffer.rays.clear();
                buffer.start_idx = i;
            }
            buffer.rays.push_back(*curr);
            last = curr;
        }

        if(buffer.rays.size() > 0) {
            segments.push_back(buffer);
        }
    }
}
