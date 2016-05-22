/// PROJECT
#include <utils_laser_processing/segmentation/ellipsoid.h>
#include <utils_laser_processing/segmentation/fit_ellipse.hpp>

#include <iostream>

using namespace lib_laser_processing;

Ellipsoid::Ellipsoid(const std::size_t support_points,
                     const double support_point_distance,
                     const double min_diameter,
                     const double max_diameter) :
    support_points_(support_points),
    support_point_distance_(support_point_distance),
    min_radius_(min_diameter),
    max_radius_(max_diameter)
{
    assert(support_points >= 2);
    assert(support_point_distance > 0.0);
    assert(min_diameter > 0.0);
    assert(max_diameter > 0.0);
}

void Ellipsoid::segmentation(const Scan &scan, std::vector<Segment> &segments)
{
    std::size_t scan_size = scan.rays.size();
    const std::vector<LaserBeam>  &rays = scan.rays;
    std::vector<Ellipse::Point>    support_points;

    for(std::size_t i = 2 ; i < scan_size - 2 ; ++i) {
        if(rays[i].invalid())
            continue;

        Ellipse::Point p = rays[i].pos();
        support_points.emplace_back(p);
        /// 1. try to find the support points
        std::size_t start = 0;
        std::size_t end = 0;
        std::size_t support_points_found = 0;
        double support_point_distance = support_point_distance_;
        for(std::size_t j = i-1 ; j > 0 ; --j) {
            const Ellipse::Point &sp = rays[j].pos();
            if(rays[j].invalid()) {
                continue;
            }
            if((sp - p).norm() > support_point_distance) {
                support_points.emplace_back(sp);
                support_point_distance += support_point_distance_;
                ++support_points_found;
                if(support_points_found >= support_points_) {
                    start = j;
                    break;
                }
            }
        }
        if(support_points_found < support_points_) {
            continue;
        }

        support_points_found = 0;
        support_point_distance = support_point_distance_;
        for(std::size_t j = i+1 ; j < scan_size ; ++j) {
            const Ellipse::Point &sp = rays[j].pos();
            if(rays[j].invalid()) {
                continue;
            }
            if((sp - p).norm() > support_point_distance) {
                support_points.emplace_back(sp);
                support_point_distance += support_point_distance_;
                ++support_points_found;
                if(support_points_found >= support_points_) {
                    end = j;
                    break;
                }
            }
        }
        if(support_points_found < support_points_) {
            continue;
        }


        /// 2. fit an ellipse
        Ellipse e;
        e.addPoints(support_points);
        e.compute();
        /// 3. evaluate results and a segment
        Ellipse::Lengths l = e.getAxisLengths();
        std::cout << l.first << " " << l.second << std::endl;
        if((l.first >= min_radius_ || l.second >= min_radius_ ) &&
                (l.first <= max_radius_ && l.second <= max_radius_)) {
            Segment s;
            for(std::size_t j = start ; j < end ; ++j) {
                if(rays[j].valid())
                    s.rays.emplace_back(rays[j]);
            }
            segments.emplace_back(s);
        }
        //        i = end;
    }
}
