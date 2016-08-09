/**
 * (c) Cognitive Systems, University of Tübingen
 *
 * @date Aug 2012
 * @author marks, buck
 */

#ifndef LEG_DETECTOR_H
#define LEG_DETECTOR_H

// C/C++
#include <vector>

// Eigen
#include <Eigen/Core>

// Project
#include <cslibs_laser_processing/data/laser_beam.h>
#include <cslibs_laser_processing/segmentation/distance_segmentation.h>
#include <cslibs_laser_processing/person_filter/leg.h>

namespace lib_laser_processing {

/**
 * @brief Very simple human leg classifier.
 */
class LegDetector
{
public:
    /**
     * @brief Create and initialize with default values.
     */
    LegDetector(unsigned int min_beams = 5,
                float max_cluster_width = 0.2,
                float max_std_dev = 0.05,
                float max_distance = 15.0);

    /**
     * @brief setParameters
     * @param min_beams Minimum number of beams to consider a leg
     * @param max_distance_from_com Maximum distance a point can have to the center
     * @param max_std_dev Maximum allowed standard deviation of segment
     * @param max_norm
     */
    void setParameters(unsigned int min_beams = 5,
                float max_cluster_width = 0.2,
                float max_std_dev = 0.05,
                float max_distance = 15.0);

    /**
     * @brief Process and classify segments
     * @param segments The new segments
     */
    void update(const std::vector<Segment> &segments );

    /**
     * @brief Classifies a segment
     * @param segment The segment to classfiy
     * @param pos Output: The position of a detected leg if one was detected
     * @return <b>true</b> iff the segment is classified as a leg
     */
    bool classify(const Segment &segments , Eigen::Vector2d &pos);

    /**
     * @brief Return all legs
     * @return All segments that have been classified as legs
     */
    const std::vector<Leg>& getLegs() const {
        return legs_;
    }

private:

    /// Minimum number of beams per leg
    unsigned int min_beams_;

    float max_cluster_width_;
    float max_std_dev_;
    float max_distance_;

    /// All segements that have been classified as legs
    std::vector<Leg> legs_;
};

}

#endif // LEG_DETECTOR_H
