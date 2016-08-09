#ifndef SEGMENTLENGTH_H
#define SEGMENTLENGTH_H

/// COMPONENT
#include <cslibs_laser_processing/segmentation/segmentation.h>

namespace lib_laser_processing {
/**
 * @brief The LineSegmentation class represents a scan segmentation algorithm
 *        that has the strategy to directly fit lines in to the scan. Those
 *        lines will be handled as segments.
 */
class SegmentLength : public LaserScanSegmentation
{
public:
    /**
     * @brief IterativeSegmentation constructor.
     * @param sigma         the maximum distance to a fitted line
     * @param max_distance      the maximum distance between to points
     */
    SegmentLength(const double max_distance,
                  const double max_length);

    /**
     * @brief ~IterativeSegmentation destructor.
     */
    virtual ~SegmentLength();

    /**
     * @brief Implemenation of the segmentation.
     */
    void segmentation(const Scan &scan, std::vector<Segment> &segments);

protected:
    double max_distance_;   /// the maximum distance points may have to each other
    double max_length_;
};
}
#endif // SEGMENTLENGTH_H
