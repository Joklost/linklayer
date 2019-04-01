
#include <common/equality.h>
#include <common/helpers.h>

#include "model.h"

const linklayer::Link linklayer::LinkModel::get_link(int x, int y, double timestamp) {
    auto &topology = this->get_topology(timestamp);

    for (const auto &link : topology.links) {
        auto &node_pair = link.nodes;
        if (((node_pair.first.id == x && node_pair.second.id == y) ||
             (node_pair.first.id == y && node_pair.second.id == x))) {
            return link;
        }
    }

    return linklayer::Link{}; /* No link found. */
}

double linklayer::LinkModel::should_receive(const Action &t, const Action &r, const std::vector<Action> &tx_list) {
    auto &link = this->get_link(t.id, r.id, t.start);
    if (link.id == 0ull || common::is_zero(link.rssi)) {
        /* No link. */
        return false;
    }

    std::vector<double> interference{};
    auto rssi = link.rssi;

    for (auto &tx_i : tx_list) {
        if (tx_i.id == t.id) {
            /* No interference from own transmission. */
            continue;
        }

        if (t.end <= tx_i.start || t.start >= tx_i.end) {
            /* Time interval does not intersect. */
            continue;
        }

        auto &link_i = this->get_link(tx_i.id, r.id, tx_i.start);
        if (link_i.id == 0ull || common::is_zero(link_i.rssi)) {
            continue;
        }

        interference.push_back(link_i.rssi);
    }

    auto pep = linklayer::pep(rssi, linklayer::PACKET_SIZE, interference);
    return pep;
}

linklayer::Topology &linklayer::LinkModel::get_topology(const double timestamp) {
    auto lower_bound = 0.0;
    for (auto &topology : this->topologies) {
        if (common::is_equal(topology.first, timestamp)) {
            lower_bound = topology.first;
            break;
        }

        if (timestamp <= topology.first && timestamp > lower_bound) {
            break;
        }

        lower_bound = topology.first;
    }

    auto &topology = this->topologies[lower_bound];

    if (topology.links.empty()) {
        /* Generate topology. */
        const auto time = topology.timestamp;
        auto &links = topology.links;

        for (unsigned long i = 0; i < this->node_list.size(); ++i) {
            auto node1 = this->node_list[i]; /* Copy */

            for (auto &location : node1.location_history) {
                if (location.get_time() <= time && location.get_time() > (time - linklayer::TIME_GAP)) {
                    node1.location = location;
                }
            }

            for (unsigned long j = i + 1; j < this->node_list.size(); ++j) {
                auto node2 = this->node_list[j]; /* Copy */

                for (auto &location : node2.location_history) {
                    if (location.get_time() <= time && location.get_time() > (time - linklayer::TIME_GAP)) {
                        node2.location = location;
                    }
                }

                if (!(node1.location.get_latitude() > 0 && node2.location.get_latitude() > 0) ||
                    !(node1.location.get_longitude() > 0 && node2.location.get_longitude() > 0)) {
                    continue;
                }

                auto it1 = node1.location.connections.find(node2.id);
                auto it2 = node2.location.connections.find(node1.id);

                if (it1 == node1.location.connections.end() || it2 == node2.location.connections.end()) {
                    continue;
                }

                auto id = common::combine_ids(node1.id, node2.id);
                links.emplace_back(id, node1, node2);
                auto &link = links.back();
                link.rssi = (it1->second + it2->second) / 2;  /* Take the average of the two. */
            }
        }
    }

    return topology;
}

linklayer::LinkModel::LinkModel(int nchans, linklayer::NodeMap n_map) : tx(nchans), rx(nchans),
                                                                        node_map(std::move(n_map)) {
    /* Generate topologies. */
    node_list.reserve(node_map.size());
    for (auto &item : node_map) {
        auto node = item.second;
        node_list.push_back(node);
    }

    for (auto &item : node_map) {
        auto &node = item.second;
        for (auto &location : node.location_history) {
            if (topologies.find(location.get_time()) == topologies.end()) {
                topologies[location.get_time()] = {location.get_time()};
            }
        }
    }
}

double linklayer::linearize(double logarithmic_value) {
    return std::pow(10, logarithmic_value / 10);
}

double linklayer::logarithmicize(double linear_value) {
    return 10 * std::log10(linear_value);
}

double linklayer::pep(double rssi, unsigned long packetsize, const std::vector<double> &interference) {
    auto P_N_dB = linklayer::THERMAL_NOISE + linklayer::NOISE_FIGURE;
    auto P_N = linearize(P_N_dB);

    auto P_I = 0.0;
    for (auto &RSSI_interference_dB : interference) {
        P_I += linearize(RSSI_interference_dB);
    }

    auto P_NI = P_N + P_I;
    auto P_NI_dB = logarithmicize(P_NI);
    auto SINR_dB = rssi - P_NI_dB;
    auto SINR = linearize(SINR_dB);

    auto bep = 0.5 * std::erfc(std::sqrt(SINR / 2.0));  /* Bit error probability. */
    auto pep = 1.0 - std::pow((1.0 - bep), packetsize * 8.0); /* Packet error probability. */
    return pep;
}
