/// HEADER
#include <utils_laser_processing/data/laser_beam.h>

using namespace lib_laser_processing;

LaserBeam::LaserBeam()
    : range_( 0 )
{}

LaserBeam::LaserBeam(const Eigen::Vector2d& pos)
    : range_( std::sqrt(pos.dot(pos)) ),
      yaw_( std::atan2(pos(1), pos(0)) ),
      pos_x_( pos(0) ),
      pos_y_( pos(1) )
{}


LaserBeam::LaserBeam(float angle, float range)
    : range_( range ),
      yaw_( angle ),
      pos_x_(std::cos(angle) * range),
      pos_y_(std::sin(angle) * range)
{}

float LaserBeam::range() const
{
    return range_;
}

float LaserBeam::yaw() const
{
    return yaw_;
}

double LaserBeam::posX() const
{
    return pos_x_;
}

double LaserBeam::posY() const
{
    return pos_y_;
}

bool LaserBeam::valid() const
{
    return range_ > 0.0;
}

bool LaserBeam::invalid() const
{
    return range_ == 0.0;
}

void LaserBeam::invalidate()
{
    range_ = 0.0;
}
