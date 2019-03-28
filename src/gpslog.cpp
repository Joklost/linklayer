#include <fstream>
#include <common/strings.h>
#include <algorithm>

#include "gpslog.h"

lm::NodeMap parse_gpsfile(const char *gpslog) {
    lm::NodeMap nodes{};
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
        auto id = std::stoul(tokens[0]);
        auto latitude = std::stod(tokens[1]);
        auto longitude = std::stod(tokens[2]);
        auto timestamp = std::stod(tokens[3]);

        auto &node = nodes[id]; /* operator[] implicitly constructs new object */
        node.id = id;
        node.location_history.emplace_back(timestamp, latitude, longitude);
    }

    logfile.close();

    for (auto &item : nodes) {
        auto &node = item.second;
        std::sort(node.location_history.begin(), node.location_history.end());
    }

    return nodes;
}
