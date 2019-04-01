#ifndef LINKLAYER_MODEL_H
#define LINKLAYER_MODEL_H

#include <utility>
#include <random>
#include <map>
#include <unordered_map>

#include <common/equality.h>

#include "node.h"
#include "link.h"
#include "action.h"

namespace linklayer {
    const int LM_ERROR = -1;

    const double TIME_GAP = 20000.0;

    const unsigned long PACKET_SIZE = 20;
    const double THERMAL_NOISE = -119.66;
    const double NOISE_FIGURE = 4.2;

    struct Topology {
        double timestamp{};
        std::vector<linklayer::Link> links{};
    };

    using NodeMap = std::unordered_map<unsigned long, linklayer::Node>;
    using NodeList = std::vector<linklayer::Node>;
    using TopologyMap = std::map<double, Topology, common::is_less<double>>;

    struct LinkModel {
        LinkModel(int nchans, NodeMap n_map);

        NodeMap node_map{};
        TopologyMap topologies{};
        NodeList node_list{};

        std::vector<std::vector<Action>> tx{};
        std::vector<std::vector<Action>> rx{};

        const linklayer::Link get_link(int x, int y, double timestamp);

        double should_receive(const Action &t, const Action &r, const std::vector<Action> &tx_list);

        Topology &get_topology(double timestamp);
    };

    double linearize(double logarithmic_value);

    double logarithmicize(double linear_value);

    double pep(double rssi, unsigned long packetsize, const std::vector<double>& interference);
}


#endif /* LINKLAYER_MODEL_H */
