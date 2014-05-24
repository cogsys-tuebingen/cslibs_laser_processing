#ifndef LABELED_SCAN_H
#define LABELED_SCAN_H

/// SYSTEM
#include <utils_laser_processing/data/scan.h>

namespace csapex
{

struct LabeledScan : public Scan
{
public:
    typedef boost::shared_ptr<LabeledScan> Ptr;

    void init(Scan& other);

public:
    std::vector<int> labels;
};

}

#endif // LABELED_SCAN_H
