#ifndef SEGMENTATION_H
#define SEGMENTATION_H

/// SYSTEM
#include <Eigen/Core>
#include <boost/shared_ptr.hpp>

/// COMPONENT
#include <utils_laser_processing/common/vector.hpp>
#include <utils_laser_processing/data/laser_beam.h>

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
     * @param reading           the range reading
     * @param angular_res       the angular resolution of the range reading
     * @param min_angle         the minimum angle of the range reading
     * @param lines             the lines that will be segmented
     */
    virtual void segmentation(const std::vector<float> &reading, const float angular_res, const float min_angle,
                              const float min_rho, const float max_rho, std::vector<std::vector<LaserBeam> > &lines) = 0;

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
