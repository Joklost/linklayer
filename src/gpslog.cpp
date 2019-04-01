#include <fstream>
#include <common/strings.h>
#include <algorithm>

#include "gpslog.h"

linklayer::NodeMap parse_gpsfile(const char *gpslog) {
    linklayer::NodeMap nodes{};
    std::ifstream logfile{gpslog};

    if (!logfile.is_open()) {
        throw std::runtime_error("failed to open gpslog file");
    }

    while (logfile.good()) {
        std::string line;
        std::getline(logfile, line);

        if (line.empty()) {
            continue;
        }

        auto tokens = common::split(line, ",");
        auto id = std::stoul(tokens.front());
        tokens.pop_front();
        auto latitude = std::stod(tokens.front());
        tokens.pop_front();
        auto longitude = std::stod(tokens.front());
        tokens.pop_front();
        auto timestamp = std::stod(tokens.front());
        tokens.pop_front();

        auto &node = nodes[id]; /* operator[] implicitly constructs new object */
        node.id = id;
        node.location_history.emplace_back(timestamp, latitude, longitude);
        auto &location = node.location_history.back();

        while (!tokens.empty()) {
            auto n_id = std::stoul(tokens.front());
            tokens.pop_front();
            auto rssi = std::stod(tokens.front());
            tokens.pop_front();
            location.connections[n_id] = rssi;
        }
    }

    logfile.close();

    for (auto &item : nodes) {
        auto &node = item.second;
        std::sort(node.location_history.begin(), node.location_history.end());
    }

    return nodes;
}
