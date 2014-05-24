#ifndef SEGMENT_H
#define SEGMENT_H

/// COMPONENT
#include <utils_laser_processing/data/laser_beam.h>

/// SYSTEM
#include <vector>
#include <Eigen/Geometry>

namespace lib_laser_processing
{

class Segment
{
public:
    Segment();

    std::vector<LaserBeam> rays;
    std::string            frame_id;

    int classification;
};

}

#endif // SEGMENT_H
