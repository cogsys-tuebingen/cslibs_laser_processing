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


    float range() const;

    float yaw() const;

    double posX() const;

    double posY() const;

    bool valid() const;

    bool invalid() const;

    void invalidate();


private:
    /// Range [m] in laser coordinate system
    float range_;

    /// Counter clockwise yaw angle, relative to x axis [rad] in laser coordinate system
    float yaw_;

    /// Position in cartesian coordinates [m x m]
    double pos_x_;
    double pos_y_;




};

} // namespace

#endif // LASERBEAM_H
