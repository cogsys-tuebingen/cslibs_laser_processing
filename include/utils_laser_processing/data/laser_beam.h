/**
 * (c) Cognitive Systems, University of Tübingen
 *
 * @author: marks
 * @date 2011
 */

#ifndef LASERBEAM_H
#define LASERBEAM_H


// Eigen
#include <Eigen/Core>

namespace lib_laser_processing {

/**
 * Represents a laser range measurement. Used to keep the core
 * algorithms independent from ROS.
 */
class LaserBeam {
public:
    /**
     * Create a laser beam. All values will be initialized to false/zero.
     */
    LaserBeam();

    /**
     * Create a laser beam. All values will be initialized from the point
     */
    LaserBeam(const Eigen::Vector2d& pos);

    /**
     * Create a laser beam. All values will be initialized from polar coordinates
     */
    LaserBeam(float angle, float range);

    /// Range [m] in laser coordinate system
    float range;

    /// Counter clockwise yaw angle, relative to x axis [rad] in laser coordinate system
    float yaw;

    /// Flag if range measurement is valid
    bool valid;

    /// Position in cartesian coordinates [m x m]
    double pos_x;
    double pos_y;
};

} // namespace

#endif // LASERBEAM_H
