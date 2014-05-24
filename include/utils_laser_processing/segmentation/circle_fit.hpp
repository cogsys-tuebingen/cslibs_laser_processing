#ifndef FIRT_CIRCLE_HPP
#define FIRT_CIRCLE_HPP

#include <pcl/point_types.h>
#include <pcl/sample_consensus/impl/ransac.hpp>
#include <pcl/sample_consensus/impl/sac_model_circle.hpp>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_circle.h>
#include <utils_laser_processing/laser_beam.h>

namespace lib_laser_processing {

using namespace pcl;

inline void fit(const std::vector<LaserBeam> &input, Eigen::Vector3d &output, double &probability) {
    PointCloud<PointXY>::Ptr cloud( new PointCloud<PointXY> );
    for(std::vector<LaserBeam>::const_iterator it = input.begin() ; it != input.end() ; ++it) {
        PointXY p;
        p.x = it->pos.x();
        p.y = it->pos.y();
        cloud->push_back(p);
    }

    SampleConsensusModelCircle2D<PointXY>::Ptr    ransac_model(new SampleConsensusModelCircle2D<PointXY>(cloud));
    SampleConsensus<PointXY>::Ptr                 ransac(new RandomSampleConsensus<PointXY>(ransac_model));
    ransac->setDistanceThreshold(0.05);

    Eigen::VectorXf coefficients;
    ransac->computeModel();
    ransac->getModelCoefficients(coefficients);
    if(coefficients.size() > 0) {
        output.x() = coefficients[0];
        output.y() = coefficients[1];
        output.z() = coefficients[2];
        probability = ransac->getProbability();
    } else {
        output.setZero();
        probability = 0.0;
    }
}
}

#endif // FIRT_CIRCLE_HPP
