/// HEADER
#include <utils_laser_processing/data/laser_beam.h>

using namespace lib_laser_processing;

LaserBeam::LaserBeam()
    : range( 0 ),
      yaw( 0 ),
      valid( false )
{}

LaserBeam::LaserBeam(const Eigen::Vector2d& pos)
    : range( std::sqrt(pos.dot(pos)) ),
      yaw( std::atan2(pos(1), pos(0)) ),
      valid( true ),
      pos_x( pos(0) ),
      pos_y( pos(1) )
{}


LaserBeam::LaserBeam(float angle, float range)
    : range( range ),
      yaw( angle ),
      valid( true ),
      pos_x(std::cos(angle) * range),
      pos_y(std::sin(angle) * range)
{}
