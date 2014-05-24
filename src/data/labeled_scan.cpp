/// HEADER
#include <utils_laser_processing/data/labeled_scan.h>

using namespace csapex;

void LabeledScan::init(Scan &other)
{
    Scan::operator= (other);

    labels.resize(ranges.size(), 0);
}
