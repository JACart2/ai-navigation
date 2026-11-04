#include <mola_input_rosbag2/Rosbag2Dataset.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/serialization/CSerializable.h>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <map>
#include <stdexcept>
#include <vector>

using GPS = mrpt::obs::CObservationGPS;
using GGA = mrpt::obs::gnss::Message_NMEA_GGA;

mrpt::maps::CSimpleMap load(const std::string& path)
{
    mrpt::maps::CSimpleMap sm;
    if (!sm.loadFromFile(path) || sm.empty()) throw std::runtime_error("Empty or unreadable SimpleMap: " + path);
    return sm;
}

void inspect(const mrpt::maps::CSimpleMap& sm)
{
    size_t gpsCount = 0, usable = 0, clouds = 0;
    uint64_t digest = 14695981039346656037ULL;
    std::vector<double> lat, lon;
    for (const auto& kf : sm)
    {
        if (!kf.sf) throw std::runtime_error("Missing sensory frame");
        for (const auto& obs : *kf.sf)
        {
            if (std::dynamic_pointer_cast<mrpt::obs::CObservationPointCloud>(obs)) ++clouds;
            const auto gps = std::dynamic_pointer_cast<GPS>(obs);
            if (!gps) continue;
            ++gpsCount;
            std::vector<uint8_t> bytes;
            mrpt::serialization::ObjectToOctetVector(gps.get(), bytes);
            for (auto b : bytes) { digest ^= b; digest *= 1099511628211ULL; }
            if (!gps->hasMsgClass<GGA>()) continue;
            const auto& g = gps->getMsgByClass<GGA>().fields;
            if (g.fix_quality == 0 || !std::isfinite(g.latitude_degrees) ||
                !std::isfinite(g.longitude_degrees) || !std::isfinite(g.altitude_meters) ||
                std::abs(g.latitude_degrees) > 90 || std::abs(g.longitude_degrees) > 180) continue;
            ++usable; lat.push_back(g.latitude_degrees); lon.push_back(g.longitude_degrees);
        }
    }
    std::sort(lat.begin(), lat.end()); std::sort(lon.begin(), lon.end());
    std::cout << "POST_PROCESSING_JSON: {\"keyframes\":" << sm.size()
              << ",\"clouds\":" << clouds << ",\"gnss_observations\":" << gpsCount
              << ",\"usable_gnss\":" << usable << ",\"gnss_digest\":\"" << digest << "\"";
    if (usable) std::cout << ",\"median_lat\":" << lat[lat.size()/2] << ",\"median_lon\":" << lon[lon.size()/2];
    std::cout << "}\n";
}

int main(int argc, char** argv) try
{
    std::cout << std::setprecision(16);
    if (argc == 3 && std::string(argv[1]) == "inspect") { inspect(load(argv[2])); return 0; }
    if (argc != 6 || std::string(argv[1]) != "attach-bag")
        throw std::runtime_error("Usage: simplemap_tools inspect MAP | attach-bag DATASET.yaml INPUT NEW_OUTPUT MAX_OFFSET_SECONDS");
    if (std::filesystem::exists(argv[4])) throw std::runtime_error("Output already exists");
    const double gate = std::stod(argv[5]);
    if (!std::isfinite(gate) || gate <= 0) throw std::runtime_error("Invalid GNSS timestamp gate");
    auto sm = load(argv[3]);
    mola::Rosbag2Dataset dataset;
    dataset.setMinLoggingLevel(mrpt::system::LVL_WARN);
    dataset.initialize(mola::Yaml::FromFile(argv[2]));
    std::map<double, GPS::Ptr> fixes;
    for (size_t i = 0; i < dataset.datasetSize(); ++i)
    {
        const auto sf = dataset.datasetGetObservations(i);
        if (!sf) continue;
        for (const auto& obs : *sf)
            if (auto gps = std::dynamic_pointer_cast<GPS>(obs))
                fixes[mrpt::Clock::toDouble(gps->timestamp)] = gps;
    }
    size_t attached = 0, missing = 0, retained = 0;
    for (auto& kf : sm)
    {
        if (kf.sf->getObservationByClass<GPS>()) { ++retained; continue; }
        const auto cloud = kf.sf->getObservationByClass<mrpt::obs::CObservationPointCloud>();
        if (!cloud) throw std::runtime_error("Keyframe without LiDAR observation");
        if (fixes.empty()) { ++missing; continue; }
        const double t = mrpt::Clock::toDouble(cloud->timestamp);
        auto hi = fixes.lower_bound(t), best = hi;
        if (hi == fixes.end()) best = std::prev(hi);
        else if (hi != fixes.begin())
        {
            const auto lo = std::prev(hi);
            if (std::abs(lo->first-t) < std::abs(hi->first-t)) best = lo;
        }
        if (std::abs(best->first-t) > gate) { ++missing; continue; }
        // The installed ROS->MRPT bridge preserves messages, covariance, timestamp,
        // fix type and antenna TF. No invented fixes, interpolation or pose changes.
        kf.sf->insert(best->second); ++attached;
    }
    if (!sm.saveToFile(argv[4])) throw std::runtime_error("Failed saving GNSS-preserved map");
    std::cout << "source_gnss=" << fixes.size() << " attached=" << attached << " retained=" << retained << " missing_within_gate=" << missing << '\n';
    inspect(sm);
    return 0;
}
catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
