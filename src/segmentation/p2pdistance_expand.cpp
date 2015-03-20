#include <utils_laser_processing/segmentation/p2pdistance_expand.h>
#include <utils_laser_processing/segmentation/utils.hpp>

#include <list>

using namespace lib_laser_processing;

P2PDistanceExpand::P2PDistanceExpand(const double max_distance) :
    max_distance_(max_distance)
{
}

P2PDistanceExpand::~P2PDistanceExpand()
{
}

void P2PDistanceExpand::segmentation(const Scan& scan, std::vector<Segment> &segments)
{
    /// PREPARATION
    segments.clear();

    /// COMPUTATION
    if(scan.rays.size() > 1) {
        std::vector<LaserBeam>::const_iterator root = scan.rays.begin();

        for(; root != scan.rays.end() ; ++root) {
            if(root->range >= scan.range_min && root->range <= scan.range_max) {
                break;
            }
        }

        std::vector<LaserBeam>::const_iterator last = root;
        std::vector<LaserBeam>::const_iterator curr = last + 1;


        Segment buffer;
        buffer.rays.push_back(*last);

        int expand = (int)asin(max_distance_ / root->range / scan.angle_increment);
        std::vector<LaserBeam>::const_iterator limit = root + expand;

        for( ; curr != scan.rays.end() ; ++curr) {
            if(fabs(last->range - curr->range) > max_distance_ || curr >= limit) {
                if(buffer.rays.size() > 1) {
                    segments.push_back(buffer);
                }
                buffer.rays.clear();
                root = curr;
                expand = (int)(asin(max_distance_ / root->range ) / scan.angle_increment);
                limit = root + expand;
            } else if(utils::distance(last, curr) < max_distance_) {
                buffer.rays.push_back(*curr);
            }
            /// outlier reduction

            last = curr;
        }

        if(buffer.rays.size() > 0) {
            segments.push_back(buffer);
        }
    }
}
