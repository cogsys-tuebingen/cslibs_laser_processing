#include <utils_laser_processing/data/laser_beam.h>

namespace lib_laser_processing {
struct LSQ {
    double sxx, syy, sxy;
    double sum_x, sum_y;
    unsigned n;

    double theta, rho, var;

    LSQ() :
        sxx(0.0), syy(0.0), sxy(0.0),
        sum_x(0.0), sum_y(0.0),
        n(0),
        theta(0.0), rho(0.0), var(0.0)
    {
    }

    void reset()
    {
        sxx = syy = sxy     = 0.0;
        sum_x = sum_y       = 0.0;
        n                   = 0;
        theta = rho = var   = 0.0;
    }

    void add(const LaserBeam &beam)
    {
        sxx   += beam.pos_x * beam.pos_x;
        syy   += beam.pos_y * beam.pos_y;
        sxy   += beam.pos_x * beam.pos_y;
        sum_x += beam.pos_x;
        sum_y += beam.pos_y;
        ++n;
    }

    void update()
    {
        const double n_inv = 1.0 / (double) n;
        const double mean_x = sum_x * n_inv;
        const double mean_y = sum_y * n_inv;
        const double sxx = this->sxx - n * mean_x * mean_x;
        const double syy = this->syy - n * mean_y * mean_y;
        const double sxy = this->sxy - n * mean_x * mean_y;
        const double theta_x = syy - sxx;
        const double theta_y = -2.0 * sxy;

        theta = 0.5 * atan2(theta_y, theta_x);
        rho   = mean_x * cos(theta) + mean_y * sin(theta);
        var   = 0.5 * n_inv * (sxx + syy - sqrt(4.0 * sxy * sxy + (syy - sxx) * (syy - sxx)));

    }

};
}
