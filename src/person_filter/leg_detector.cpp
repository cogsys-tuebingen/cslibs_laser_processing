/**
 * (c) Cognitive Systems, University of Tübingen
 *
 * @date Aug 2012
 * @author marks, buck
 */

// C/C++
#include <limits>

// Eigen
#include <Eigen/Core>

// Boost
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>

// Project
#include <cslibs_laser_processing/person_filter/leg_detector.h>

using namespace Eigen;
using namespace boost::accumulators;

namespace lib_laser_processing {

LegDetector::LegDetector(unsigned int min_beams, float max_cluster_width, float max_std_dev, float max_distance)
    : min_beams_( min_beams ), max_cluster_width_( max_cluster_width ),  max_std_dev_( max_std_dev), max_distance_( max_distance )
{
}

void LegDetector::setParameters(unsigned int min_beams, float max_cluster_width, float max_std_dev, float max_distance)
{
    min_beams_ = min_beams;
    max_cluster_width_ = max_cluster_width;
    max_std_dev_ = max_std_dev;
    max_distance_ = max_distance;
}

void LegDetector::update( const std::vector<Segment>& segments )
{
    legs_.clear();

    // For each segment
    for ( std::vector<Segment>::const_iterator it = segments.begin(); it != segments.end(); ++it ) {
        const Segment& segment = *it;

        Vector2d com;
        bool is_leg = classify(segment, com);

        if(is_leg) {
            Leg leg;
            leg.pos = com;
            leg.is_single_leg = false; /// @todo Think about this
            legs_.push_back( leg );
        }
    }
}

bool LegDetector::classify(const Segment &segment, Vector2d& pos)
{
    // Enough beams?
    if ( segment.rays.size() < min_beams_ ) {
        return false;
    }

    /// @todo Simple check if a segment is far too large

    // Compute center of mass (COM)
    Vector2d com( Vector2d::Zero());
    for ( std::vector<LaserBeam>::const_iterator beam = segment.rays.begin(); beam != segment.rays.end(); ++beam ) {
        com += Eigen::Vector2d(beam->posX(), beam->posY());
    }

    com /= static_cast<double> (segment.rays.size());

    // Compute maximum and average distance to COM
    double max = 0;
    double avr = 0;
    double d;
    double count = 0;
    for ( std::vector<LaserBeam>::const_iterator beam = segment.rays.begin(); beam != segment.rays.end(); ++beam ) {
        if ( !beam->valid() ) {
            continue;
        }

        d = (com - Eigen::Vector2d(beam->posX(), beam->posY())).norm();
        avr += d;
        if ( max < d ) {
            max = d;
        }
        count++;
    }
    avr /= count;

    // Check maximum distance
    if ( max > max_cluster_width_ )
        return false;

    // Estimated leg center
    Vector2d center( com + 0.04*com.normalized());

    /*double min_c = std::numeric_limits<double>::max();
    double max_c = 0;
    double avr_c = 0;
    count = 0;
    it.resetBeams();
    while ( it.nextBeam()) {
        if ( !it.beam().valid )
            continue;

        d = (center - it.beam().pos).norm();
        avr_c += d;
        count++;
        if ( min_c > d ) min_c = d;
        if ( max_c < d ) max_c = d;
    }
    avr_c /= count;*/

    // Standard deviation
    accumulator_set<double, stats<tag::variance> > dists;
    for ( std::vector<LaserBeam>::const_iterator beam = segment.rays.begin(); beam != segment.rays.end(); ++beam ) {
        if ( !beam->valid() ) {
            continue;
        }
        dists((center - Eigen::Vector2d(beam->posX(), beam->posY())).norm());
    }
    double std_dev = std::sqrt( variance( dists ));

    // Check std dev
    if ( std_dev > max_std_dev_ )
        return false;

    /// @todo This is a hack. Set a proper maximum distance somewhere
    if ( com.norm() > max_distance_ )
        return false;

    pos = com;

    return true;
}
} // namespace
