#ifndef CONVERSION_HPP
#define CONVERSION_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <utils_laser_processing/data/laser_beam.h>

namespace lib_laser_processing {
namespace utils {
/**
 * @brief Retrieve the cartesian point coordinates of a range reading.
 * @param reading           the range reading
 * @param angular_res       the angular resolution of the range reading
 * @param min_angle         the minimum angle of the range reading
 * @param points            the point coordiantes in cartesian space
 */
inline void polarToCartesian(const std::vector<float> &reading, const float angular_res, const float min_angle,
                             const float min_rho, const float max_rho, std::vector<LaserBeam> &points)
{
    points.clear();
    double angle = min_angle;
    for(std::vector<float>::const_iterator it = reading.begin() ; it != reading.end() ; ++it, angle += angular_res) {
        float rho = *it;
        if(rho < max_rho && rho > min_rho && rho > 0.0) {
            Eigen::Vector2d pt;
            pt.x() = std::cos(angle) * rho;
            pt.y() = std::sin(angle) * rho;
            points.push_back(pt);
        }

    }
}

/**
 * @brief Retrieve the cartesian point coordinates of a range reading.
 * @param reading           the range reading
 * @param angular_res       the angular resolution of the range reading
 * @param min_angle         the minimum angle of the range reading
 * @param points            the point coordiantes in cartesian space
 */
inline void polarToCartesian(const std::vector<float> &reading, const float angular_res, const float min_angle,
                             const float min_rho, const float max_rho, std::vector<double> &points_x, std::vector<double> &points_y)
{
    points_x.clear();
    points_y.clear();
    double angle = min_angle;
    for(std::vector<float>::const_iterator it = reading.begin() ; it != reading.end() ; ++it, angle += angular_res) {
        float rho = *it;
        if(rho < max_rho && rho > min_rho) {
            points_x.push_back( std::cos(angle) * rho);
            points_y.push_back(std::sin(angle) * rho);
        }

    }
}

/**
 * @brief Calculate the distance between two Eigen vectors given by iterators.
 * @param first             the first Eigen Vector2d
 * @param second            the second Eigen Vector2d
 * @return                  the distance
 */
inline double distance(const std::vector<LaserBeam>::const_iterator &first, const std::vector<LaserBeam>::const_iterator &second)
{
    return (Eigen::Vector2d(first->pos_x, first->pos_y) - Eigen::Vector2d(second->pos_x, second->pos_y)).norm();
}

/**
 * @brief Check if all points between first and second are close enough to the fitted line.
 * @param first             start iterator of point set
 * @param second            end   iterator of point set
 * @param line              the fitted line
 * @param sigma             the maximum distance to consider a point an inlyer
 * @return
 */
inline bool withinLineFit(const std::vector<LaserBeam>::const_iterator &first, const std::vector<LaserBeam>::const_iterator &second,
                          const Eigen::ParametrizedLine<double, 2> &line, const double sigma)
{
    assert(std::distance(first, second) > 0);

    bool fitting = true;
    for(std::vector<LaserBeam>::const_iterator it = first; it != second ; ++it) {
        fitting &= (line.distance(Eigen::Vector2d(it->pos_x, it->pos_y)) < sigma);
    }

    return fitting;
}

/**
 * @brief The 'regression2D' method can be used to fit a line to a given point set.
 * @param points_begin      set begin iterator
 * @param points_end        set end iterator
 * @param fit_start         the start of the line fit
 * @param fit_end           the set termintating iterator position
 * @param line              the parameterized line to work with
*/
inline void regression2D(const std::vector<LaserBeam>::const_iterator &points_begin, const std::vector<LaserBeam>::const_iterator &points_end,
                         Eigen::ParametrizedLine<double, 2> &line)
{
    std::vector<LaserBeam>::const_iterator back_it = points_end;
    --back_it;
    Eigen::Vector2d front (points_begin->pos_x, points_end->pos_y);
    Eigen::Vector2d back (back_it->pos_x, back_it->pos_y);

    unsigned int size = std::distance(points_begin, points_end);
    Eigen::MatrixXd A(size, 2);
    Eigen::VectorXd b(size);
    A.setOnes();

    Eigen::Vector2d dxy = (front - back).cwiseAbs();
    bool solve_for_x = dxy.x() > dxy.y();
    if(solve_for_x) {
        /// SOLVE FOR X
        int i = 0;
        for(std::vector<LaserBeam>::const_iterator it = points_begin ; it != points_end ; ++it, ++i)
        {
            A(i,1) = it->pos_x;
            b(i)   = it->pos_y;
        }
    } else {
        /// SOLVE FOR Y
        int i = 0;
        for(std::vector<LaserBeam>::const_iterator it = points_begin ; it != points_end ; ++it, ++i)
        {
            A(i,1) = it->pos_y;
            b(i)   = it->pos_x;
        }
    }

    Eigen::VectorXd weights = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
    double          alpha   = weights(0);
    double          beta    = weights(1);
    Eigen::Vector2d from;
    Eigen::Vector2d to;

    if(solve_for_x) {
        from(0) = 0.0;
        from(1) = alpha;
        to(0)   = 1.0;
        to(1)   = alpha + beta;
    } else {
        from(0) = alpha;
        from(1) = 0.0;
        to(0)   = alpha + beta;
        to(1)   = 1.0;
    }

    Eigen::Vector2d fit_start;
    Eigen::Vector2d fit_end;
    line = Eigen::ParametrizedLine<double, 2>(from, (to - from).normalized());
    fit_start = line.projection(front);
    fit_end   = line.projection(back);
    line = Eigen::ParametrizedLine<double, 2>(fit_start, (fit_end - fit_start));
}

/**
 * @brief The 'regression2D' method can be used to fit a line to a given point set.
 * @param points            the points to fit
 * @param fit_start         the start of the line fit
 * @param fit_end           the end of the line fit
 * @param line              the parameterized line to work with
 */
inline void regression2D(const std::vector<LaserBeam> &points, Eigen::ParametrizedLine<double, 2> line)
{
    regression2D(points.begin(), points.end(), line);
}
}
}


#endif // CONVERSION_HPP
