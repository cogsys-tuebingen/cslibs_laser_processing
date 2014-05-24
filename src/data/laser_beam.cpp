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
      pos( pos )
{}


LaserBeam::LaserBeam(float angle, float range)
    : range( range ),
      yaw( angle ),
      valid( true ),
      pos( Eigen::Vector2d(std::cos(angle) * range, std::sin(angle) * range) )
{}
