/**
 * (c) Cognitive Systems, University of Tübingen
 *
 * @date Aug 2012
 * @author marks, buck
 */

// Project
#include <utils_laser_processing/segmentation/distance_segmentation.h>

namespace lib_laser_processing {

DistanceSegmentation::DistanceSegmentation( float threshold )
    : threshold_( threshold )
{}

void DistanceSegmentation::segmentation(const Scan& scan, std::vector<Segment> &segments)
{
    Segment s;
    int start = 0;
    int length = 1;

    // For all beams
    bool in_segment = true;
    int end = scan.rays.size() - 1;
    for ( int i = 0; i < end - 1; ++i ) {
        // End of segment?
        if ( !scan.rays[i+1].valid()
             || std::abs( scan.rays[i].range() - scan.rays[i+1].range() ) > threshold_ ) {
            segments.push_back( s );
            start = i + 1;
            length = 1;
            in_segment = false;
            s = Segment();
            continue;
        }

        // Still in segment
        ++length;
        in_segment = true;
    }

    // Close last segment?
    if ( in_segment )
        segments.push_back( s );
}

} // namespace
