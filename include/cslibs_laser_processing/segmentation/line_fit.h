#ifndef LINE_FIT_H
#define LINE_FIT_H

/// COMPONENT
#include <cslibs_laser_processing/segmentation/segmentation.h>

namespace lib_laser_processing {
/**
 * @brief The LineFit class represents a scan segmentation algorithm, that uses a direct line
 *        fit by applying a svd decomposition with the least squares method.
 */
class LineFit : public LaserScanSegmentation
{
public:
    /**
     * @brief LineFit constructor.
     * @param sigma             the maximum distance of a point to the fitted line
     * @param max_distance      the maximum distance between to points
     */
    LineFit(const double sigma, const double max_distance);
    /**
     * @brief ~LineFit destructor.
     */
    virtual ~LineFit();

    /**
     * @brief Implemenation of the segmentation.
     */
    void segmentation(const Scan &scan, std::vector<Segment> &segments);

protected:
    double sigma_;          /// the maximum possible variance of points around the fitted line
    double max_distance_;   /// the maximum distance points may have to each other

    void pushbackLineSegment(Eigen::ParametrizedLine<double, 2> line, std::vector<Segment> &segments);
};
}
#endif // LINE_FIT_H
