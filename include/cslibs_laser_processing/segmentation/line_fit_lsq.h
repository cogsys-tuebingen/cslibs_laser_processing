#ifndef LINE_FIT_LSQ_H
#define LINE_FIT_LSQ_H

/// COMPONENT
#include <cslibs_laser_processing/segmentation/segmentation.h>
#include "lsq.hpp"

namespace lib_laser_processing {
/**
 * @brief The LineFit class represents a scan segmentation algorithm, that uses a direct line
 *        fit using least squares directly.
 */
class LineFitLSQ : public LaserScanSegmentation
{
public:
    /**
     * @brief LineFitLSQ constructor.
     * @param delta_d        the maximum distance of a point to the fitted line
     * @param delta_var      the maximum variance within line fit
     */
    LineFitLSQ(const double delta_d, const double delta_var);
    /**
     * @brief ~LineFitLSQ destructor.
     */
    virtual ~LineFitLSQ();

    /**
     * @brief Implemenation of the segmentation.
     */
    void segmentation(const Scan &scan, std::vector<Segment> &segments);

protected:
    LSQ    lsq_;
    double delta_d_;        /// point to point distance threshold for intial point sets
    double delta_var_;      /// variance threshold for line fit
};
}
#endif // LINE_FIT_H
