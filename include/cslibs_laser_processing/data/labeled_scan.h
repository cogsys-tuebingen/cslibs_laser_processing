#ifndef LABELED_SCAN_H
#define LABELED_SCAN_H

/// SYSTEM
#include <cslibs_laser_processing/data/scan.h>

namespace lib_laser_processing
{

struct LabeledScan : public Scan
{
public:
    typedef boost::shared_ptr<LabeledScan> Ptr;

    void init(const Scan &other);

public:
    std::vector<int> labels;
};

}

#endif // LABELED_SCAN_H
