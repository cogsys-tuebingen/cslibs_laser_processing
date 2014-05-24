#ifndef P2PDISTANCE_H
#define P2PDISTANCE_H

/// COMPONENT
#include <utils_laser_processing/segmentation/segmentation.h>

/**
 * @brief The P2PDistancesegmentation class represents an algorithm that
 *        processes a scan segmentation by calculating the distance between
 *        neighboured points. If a maximum distance is reached, the scan will be
 *        split.
 */
namespace lib_laser_processing {
class P2PDistance : public LaserScanSegmentation
{
public:
    /**
     * @brief P2PDistancesegmentation constructor.
     * @param max_distance      the maximum distance that may be reached
     */
    P2PDistance(const double max_distance);

    /**
     * ~P2PDistancesegmentation desctructor.
     */
    virtual ~P2PDistance();

    /**
     * @brief Implementation of the segmentation.
     */
    void segmentation(const std::vector<float> &reading, const float angular_res, const float min_angle,
                      const float min_rho, const float max_rho, std::vector<std::vector<LaserBeam> > &segments);

protected:
    double  max_distance_;      /// the maximum distance between two scan points
};
}
#endif // P2PDISTANCE_H
