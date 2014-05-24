#ifndef ITERATIVE_SEGMENTATION_H
#define ITERATIVE_SEGMENTATION_H

/// COMPONENT
#include <utils_laser_processing/segmentation/segmentation.h>

namespace lib_laser_processing {
/**
 * @brief The LineSegmentation class represents a scan segmentation algorithm
 *        that has the strategy to directly fit lines in to the scan. Those
 *        lines will be handled as segments.
 */
class P2PLine : public LaserScanSegmentation
{
public:
    /**
     * @brief IterativeSegmentation constructor.
     * @param sigma         the maximum distance to a fitted line
     * @param max_distance      the maximum distance between to points
     */
    P2PLine(const double sigma, const double max_distance);

    /**
     * @brief ~IterativeSegmentation destructor.
     */
    virtual ~P2PLine();

    /**
     * @brief Implemenation of the segmentation.
     */
    void segmentation(const std::vector<float> &reading, const float angular_res, const float min_angle,
                      const float min_rho, const float max_rho, std::vector<std::vector<LaserBeam> > &lines);

protected:
    double sigma_;          /// the maximum possible variance of points around the fitted line
    double max_distance_;   /// the maximum distance points may have to each other
};
}
#endif // ITERATIVE_SEGMENTATION_H
