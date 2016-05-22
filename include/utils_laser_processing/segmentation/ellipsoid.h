#ifndef ELLIPSOID_H
#define ELLIPSOID_H

/// PROJECT
#include <utils_laser_processing/segmentation/segmentation.h>


namespace lib_laser_processing {

class Ellipsoid : public LaserScanSegmentation
{
public:
    Ellipsoid(const std::size_t support_points,
              const double support_point_distance,
              const double min_diameter,
              const double max_diameter);

    void segmentation(const Scan &scan, std::vector<Segment> &segments) override;

private:
    std::size_t support_points_;
    double      support_point_distance_;
    double      min_radius_;    /// actually the axis length
    double      max_radius_;    /// actually the axis length

};

}

#endif // ELLIPSOID_H
