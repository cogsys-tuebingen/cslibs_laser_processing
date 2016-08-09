#ifndef SCAN_H
#define SCAN_H

/// COMPONENT
#include <cslibs_laser_processing/data/laser_beam.h>

/// SYSTEM
#include <vector>
#include <string>
#include <boost/shared_ptr.hpp>

namespace lib_laser_processing
{

struct Scan
{
public:
    typedef boost::shared_ptr<Scan> Ptr;

public:
    Scan();
    Scan(const std::vector<float>& ranges, float angle_min, float angle_increment, float min_range, float max_range);

    void getRanges(std::vector<float> &out) const;

public:
    struct Header {
        unsigned int seq;

        unsigned long stamp_nsec;

        std::string frame_id;
    };

    Header header;

    float angle_min;
    float angle_max;

    float angle_increment;

    float range_min;

    float range_max;

    std::vector<LaserBeam> rays;

    bool valid;
};

}

#endif // SCAN_H
