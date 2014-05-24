/// HEADER
#include <utils_laser_processing/data/labeled_scan.h>

using namespace lib_laser_processing;

void LabeledScan::init(Scan &other)
{
    Scan::operator= (other);

    labels.resize(rays.size(), 0);
}
