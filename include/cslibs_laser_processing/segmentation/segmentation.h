#ifndef SEGMENTATION_H
#define SEGMENTATION_H

/// SYSTEM
#include <Eigen/Core>
#include <boost/shared_ptr.hpp>

/// COMPONENT
#include <cslibs_laser_processing/common/vector.hpp>
#include <cslibs_laser_processing/data/laser_beam.h>
#include <cslibs_laser_processing/data/scan.h>
#include <cslibs_laser_processing/data/segment.h>

/**
 * @brief The LaserScanSegmentation class is representing an interface for computing laser scans,
 *        building segmentations to estimate lines.
 */
namespace lib_laser_processing {
class LaserScanSegmentation {
public:
    /// SHARED POINTER
    typedef boost::shared_ptr<LaserScanSegmentation> Ptr;

    /**
     * @brief Segmentation of a laser range reading.
     * @param scan     the range reading
     * @param segments the lines that will be segmented
     */
    virtual void segmentation(const Scan& scan, std::vector<Segment> &segments) = 0;

    /**
     * @brief ~Scan2DSegmentation destructor.
     */
    virtual ~LaserScanSegmentation();

protected:
    /**
     * @brief Scan2DSegmentation private constructor to avoid instantiation of the interface.
     */
    LaserScanSegmentation();



};
}
#endif // SEGMENTATION_H
