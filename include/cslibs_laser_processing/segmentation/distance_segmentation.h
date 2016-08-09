/**
 * (c) Cognitive Systems, University of Tübingen
 *
 * @date Aug 2012
 * @author marks, buck
 */

#ifndef DISTANCE_SEGMENTATION_H
#define DISTANCE_SEGMENTATION_H

// C/C++
#include <vector>

// Project
#include <cslibs_laser_processing/data/laser_beam.h>
#include <cslibs_laser_processing/data/scan.h>
#include <cslibs_laser_processing/segmentation/segmentation.h>

namespace lib_laser_processing {

/// @todo Extract a parent class to make switching between different algorithms easier

/**
 * @brief Provides a simple distance based segmentation of laser data
 */
class DistanceSegmentation  : public LaserScanSegmentation
{
public:

    /**
     * @brief Create and initialize
     * @param threshold Distance threshold
     */
    DistanceSegmentation( float threshold );

    /**
     * @brief Implementation of the segmentation.
     */
    void segmentation(const Scan &scan, std::vector<Segment> &segments);

private:

    /// Distance threshold [m]
    float threshold_;

};

} // namespace

#endif // DISTANCE_SEGMENTATION_H
