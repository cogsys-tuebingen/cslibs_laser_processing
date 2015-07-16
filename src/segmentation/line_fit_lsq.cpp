/// HEADER
#include <utils_laser_processing/segmentation/line_fit_lsq.h>

/// COMPONENT
#include <utils_laser_processing/segmentation/utils.hpp>

#include <iostream>

using namespace lib_laser_processing;

LineFitLSQ::LineFitLSQ(const double delta_d, const double delta_var) :
    delta_d_(delta_d),
    delta_var_(delta_var)
{
}

LineFitLSQ::~LineFitLSQ()
{
}

namespace distance {
inline double euclidean(const LaserBeam &b1,
                        const LaserBeam &b2)
{
    return hypot(b1.pos_x - b2.pos_x, b1.pos_y - b2.pos_y);
}
inline double line(const LaserBeam &b,
                   const LSQ &lsq)
{
    return fabs(b.pos_x * cos(lsq.theta) +
                b.pos_y * sin(lsq.theta) -
                lsq.rho);
}
}

void LineFitLSQ::segmentation(const Scan& scan, std::vector<Segment> &segments)
{
    lsq_.reset();
    segments.clear();

    const LaserBeam *beams = scan.rays.data();

    unsigned int k_0 = 0;
    unsigned int k_1 = k_0+1;
    unsigned int k_2 = k_0+2;
    unsigned int n = scan.rays.size();

    Segment buffer;
    while(k_2 < n) {
        /// FIND INITIAL POINTS
        buffer.rays.clear();
        while(k_2 < n) {
            if(distance::euclidean(beams[k_1], beams[k_0]) < delta_d_ &&
               distance::euclidean(beams[k_2], beams[k_1]) < delta_d_) {
                lsq_.add(beams[k_0]);
                lsq_.add(beams[k_1]);
                lsq_.add(beams[k_2]);
                lsq_.update();
                if(lsq_.var < delta_var_) {
                    buffer.rays.push_back(beams[k_0]);
                    buffer.rays.push_back(beams[k_1]);
                    buffer.rays.push_back(beams[k_2]);
                    break;
                }
                lsq_.reset();
            }
            ++k_0;++k_1;++k_2;
        }
        /// APPEND TO LINE WHEN CONDITIONS ARE FULFILLED
        while(k_2 < n) {
            ++k_2;

            if(beams[k_2].range > scan.range_max ||
                    beams[k_2].range < scan.range_min)
                continue;

            ++k_0;++k_1;

            if(distance::euclidean(beams[k_2], beams[k_1]) < delta_d_ &&
               distance::line(beams[k_2], lsq_) < delta_d_) {
                lsq_.add(beams[k_2]);
                lsq_.update();
                buffer.rays.push_back(beams[k_2]);
            } else {
                segments.push_back(buffer);
                lsq_.reset();
                buffer.rays.clear();

                k_0 = k_2;
                k_1 = k_0 + 1;
                k_2 = k_0 + 2;
                break;
            }
        }
    }
    if(buffer.rays.size() > 0)
        segments.push_back(buffer);
}
