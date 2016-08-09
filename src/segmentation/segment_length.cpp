#include <cslibs_laser_processing/segmentation/segment_length.h>
#include <cslibs_laser_processing/segmentation/utils.hpp>

using namespace lib_laser_processing;

SegmentLength::SegmentLength(const double max_distance,
                             const double max_length) :
    max_distance_(max_distance),
    max_length_(max_length)
{
}

SegmentLength::~SegmentLength()
{
}

void SegmentLength::segmentation(const Scan& scan, std::vector<Segment> &segments)
{
    /// PREPARATION
    segments.clear();

    /// COMPUTATION
    if(scan.rays.size() > 1) {
        std::vector<LaserBeam>::const_iterator last = scan.rays.begin();
        while(last->invalid() && last != scan.rays.end())
            ++last;

        if(last == scan.rays.end())
            return;

        std::vector<LaserBeam>::const_iterator curr = last + 1;
        Segment buffer;
        buffer.rays.push_back(*last);
        double length = 0;
        for( ; curr != scan.rays.end() ; ++curr) {
            if(curr->invalid())
                continue;

            double distance = utils::distance(last, curr);
            if(distance < max_distance_) {
                buffer.rays.push_back(*curr);
                length += distance;
            } else {
                if(length < max_length_) {
                    segments.push_back(buffer);
                }
                buffer.rays.clear();
                length = 0.0;
            }

            last = curr;
        }

        if(buffer.rays.size() > 0) {
            segments.push_back(buffer);
        }
    }
}
