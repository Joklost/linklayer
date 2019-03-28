
#include <common/equality.h>
#include <common/helpers.h>
#include <sims/radiomodel.h>
#include <sims/math.h>

#include "model.h"

bool lm::Action::is_within(const Action &action) const {
    return this->start >= action.start && this->end <= action.end;
}

bool lm::Action::operator==(const lm::Action &rhs) const {
    return state == rhs.state &&
           id == rhs.id &&
           chn == rhs.chn;
//         &&  start == rhs.start;
}

bool lm::Action::operator!=(const lm::Action &rhs) const {
    return !(rhs == *this);
}

lm::Action::Action(lm::State state, int id, int chn) : state(state), id(id), chn(chn) {}

lm::Action::Action(lm::State state, int id, int chn, double start) : state(state), id(id),
                                                                                            chn(chn), start(start) {}

lm::Action::Action(lm::State state, int id, int chn, double start, double end) : state(
        state), id(id), chn(chn), start(start), end(end) {}

lm::Action::Action() = default;

const sims::Link lm::LinkModel::get_link(int x, int y, double timestamp) {
    auto &topology = this->get_topology(timestamp);

    for (const auto &link : topology.links) {
        auto &node_pair = link.get_nodes();
        if (((node_pair.first.get_id() == x && node_pair.second.get_id() == y) ||
             (node_pair.first.get_id() == y && node_pair.second.get_id() == x))) {
            return link;
        }
    }

    return sims::Link{}; /* No link found. */
}

double lm::LinkModel::should_receive(const Action &t, const Action &r, const std::vector<Action> &tx_list) {
    auto &link = this->get_link(t.id, r.id, t.start);
    if (link.get_id() == 0ull || common::is_zero(link.get_distance())) {
        /* No link. */
        return false;
    }

    std::vector<double> interference{};
    auto rssi = lm::TX_POWER - link.get_distance();

    for (auto &tx_i : tx_list) {
        if (tx_i.id == t.id) {
            /* No interference from own transmission. */
            continue;
        }

        if (r.end <= tx_i.start || r.start >= tx_i.end) {
            /* Time interval does not intersect. */
            continue;
        }

        auto &link_i = this->get_link(t.id, r.id, t.start);
        if (link.get_id() == 0ul || common::is_zero(link.get_distance())) {
            continue;
        }

        interference.push_back(lm::TX_POWER - link_i.get_distance());
    }

//    std::random_device rd{};
//    std::mt19937 gen{rd()};
//    auto pep = sims::radiomodel::pep(rssi, lm::PACKET_SIZE, interference);
    return sims::radiomodel::pep(rssi, lm::PACKET_SIZE, interference);
//    std::bernoulli_distribution d(1.0 - pep);
//    return d(gen);
}

lm::Topology &lm::LinkModel::get_topology(const double timestamp) {
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
                if (location.get_time() <= time && location.get_time() > (time - lm::TIME_GAP)) {
                    node1.current_location = location;
                }
            }

            for (unsigned long j = i + 1; j < this->node_list.size(); ++j) {
                auto node2 = this->node_list[j]; /* Copy */

                for (auto &location : node2.location_history) {
                    if (location.get_time() <= time && location.get_time() > (time - lm::TIME_GAP)) {
                        node2.current_location = location;
                    }
                }

                if (node1.current_location.get_latitude() > 0 && node2.current_location.get_latitude() > 0 &&
                    node1.current_location.get_longitude() > 0 && node2.current_location.get_longitude() > 0) {
                    auto id = common::combine_ids(node1.get_id(), node2.get_id());
                    links.emplace_back(id, node1, node2);
                    auto &link = links.back();
                    /* Compute distance based path loss on the links as we create them. */
                    auto distance = geo::distance_between(node1.current_location, node2.current_location);
                    auto pathloss = sims::math::distance_pathloss(distance * KM);
                    link.distance = pathloss;
                }
            }
        }
    }

    return topology;
}

lm::LinkModel::LinkModel(int nchans, lm::NodeMap n_map) : tx(nchans), rx(nchans), node_map(std::move(n_map)) {
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
