#ifndef LASER_SCAN_YAMLIO_HPP
#define LASER_SCAN_YAMLIO_HPP

/// PROJECT
#include <cslibs_laser_processing/data/segment.h>
#include <cslibs_laser_processing/data/laser_beam.h>
#include <cslibs_laser_processing/data/scan.h>
#include <cslibs_laser_processing/data/labeled_scan.h>

/// SYSTEM
#include <yaml-cpp/yaml.h>

/// YAML
namespace YAML {

template <typename Fp, typename Key>
inline Fp readFloatSafe(const Node& node, const Key& key) {
    try {
        return node[key].template as<Fp>();
    } catch(const YAML::BadConversion& e) {
        std::string str = node[key].template as<std::string>();
        if(str == "inf") {
            return std::numeric_limits<Fp>::infinity();
        } else if(str == "-inf") {
            return -std::numeric_limits<Fp>::infinity();
        } else if(str == "nan") {
            return std::numeric_limits<Fp>::signaling_NaN();
        } else if(str == "-nan") {
            return -std::numeric_limits<Fp>::signaling_NaN();
        } else {
            throw;
        }
    }
}

template<>
struct convert<lib_laser_processing::LaserBeam> {
    static Node encode(const lib_laser_processing::LaserBeam& rhs) {
        Node node;
        node.push_back(rhs.range());
        node.push_back(rhs.yaw());
        return node;
    }

    static bool decode(const Node& node, lib_laser_processing::LaserBeam& rhs) {
        if(!node.IsSequence()) {
            return false;
        }

        float  range  = readFloatSafe<float>(node, 0);
        float  yaw    = readFloatSafe<float>(node, 1);
        rhs = lib_laser_processing::LaserBeam(yaw, range);


        return true;
    }
};

template<>
struct convert<lib_laser_processing::Segment> {
    static Node encode(const lib_laser_processing::Segment& rhs) {
        Node node;
        node.push_back(rhs.frame_id);
        node.push_back(rhs.rays);
        node.push_back(rhs.classification);
        node.push_back(rhs.stamp_micro_seconds);
        return node;
    }

    static bool decode(const Node& node, lib_laser_processing::Segment& rhs) {
        if(!node.IsSequence() || node.size() != 3) {
            return false;
        }

        rhs.frame_id = node[0].as<std::string>();
        rhs.rays = node[1].as<std::vector<lib_laser_processing::LaserBeam> >();
        rhs.classification = node[2].as<int>();
        rhs.stamp_micro_seconds = node[3].as<uint64_t>();
        return true;
    }
};


template<>
struct convert<lib_laser_processing::Scan> {
    static Node encode(const lib_laser_processing::Scan& rhs)
    {
        Node node;

        node["header/seq"] = rhs.header.seq;
        node["header/stamp"] = rhs.header.stamp_nsec;
        node["header/frame_id"] = rhs.header.frame_id;

        node["angle_min"] = rhs.angle_min;
        node["angle_max"] = rhs.angle_max;
        node["angle_increment"] = rhs.angle_increment;
        node["range_min"] = rhs.range_min;
        node["range_max"] = rhs.range_max;

        node["ranges"] = rhs.rays;
        node["valid"] = rhs.valid;
        return node;
    }

    static bool decode(const Node& node, lib_laser_processing::Scan& rhs)
    {
        if(!node.IsMap()) {
            return false;
        }

        rhs.header.seq = node["header/seq"].as<unsigned int>();
        rhs.header.stamp_nsec = node["header/stamp"].as<unsigned long>();
        rhs.header.frame_id = node["header/frame_id"].as<std::string>();

        rhs.angle_min = readFloatSafe<float>(node, "angle_min");
        rhs.angle_max = readFloatSafe<float>(node, "angle_max");
        rhs.angle_increment = readFloatSafe<float>(node, "angle_increment");
        rhs.range_min = readFloatSafe<float>(node, "range_min");
        rhs.range_max = readFloatSafe<float>(node, "range_max");

        rhs.range_min = std::max(rhs.range_min, 0.01f);

        rhs.rays = node["ranges"].as<std::vector<lib_laser_processing::LaserBeam> >();
        rhs.valid = node["valid"].as<bool>();
        return true;
    }
};

template<>
struct convert<lib_laser_processing::LabeledScan> {
    static Node encode(const lib_laser_processing::LabeledScan& rhs)
    {
        Node node = convert<lib_laser_processing::Scan>::encode(rhs);
        node["labels"] = rhs.labels;

        return node;
    }

    static bool decode(const Node& node, lib_laser_processing::LabeledScan& rhs)
    {
        if(!node.IsMap()) {
            return false;
        }
        convert<lib_laser_processing::Scan>::decode(node, rhs);

        rhs.labels = node["labels"].as<std::vector<int> >();
        return true;
    }
};
}
#endif // LASER_SCAN_YAMLIO_HPP
