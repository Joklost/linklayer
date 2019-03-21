
#include <common/equality.h>
#include <reachi/radiomodel.h>
#include "model.h"

bool linkaiders::Action::is_within(const Action &action) const {
    return this->start >= action.start && this->end <= action.end;
}

bool linkaiders::Action::operator==(const linkaiders::Action &rhs) const {
    return state == rhs.state &&
           id == rhs.id &&
           chn == rhs.chn;
//         &&  start == rhs.start;
}

bool linkaiders::Action::operator!=(const linkaiders::Action &rhs) const {
    return !(rhs == *this);
}

linkaiders::Action::Action(linkaiders::State state, int id, int chn) : state(state), id(id), chn(chn) {}

linkaiders::Action::Action(linkaiders::State state, int id, int chn, double start) : state(state), id(id),
                                                                                            chn(chn), start(start) {}

linkaiders::Action::Action(linkaiders::State state, int id, int chn, double start, double end) : state(
        state), id(id), chn(chn), start(start), end(end) {}

linkaiders::Action::Action() = default;

const reachi::Link linkaiders::LinkModel::get_link(int x, int y, double timestamp) const {
    auto topology = this->topologies.lower_bound(timestamp);
    if (topology == this->topologies.end()) {
        return reachi::Link{}; /* No link found. */
    }

    auto &links = topology->second.links;

    for (const auto &link : links) {
        auto &node_pair = link.get_nodes();
        if (((node_pair.first.get_id() == x && node_pair.second.get_id() == y) ||
             (node_pair.first.get_id() == y && node_pair.second.get_id() == x))) {
            return link;
        }
    }

    return reachi::Link{}; /* No link found. */
}

bool linkaiders::LinkModel::should_receive(const Action &t, const Action &r) {
    auto &link = this->get_link(t.id, r.id, t.start);
    if (link.get_id() == 0ul || common::is_zero(link.get_distance())) {
        /* No link. */
        return false;
    }

    std::vector<double> interference{};
    auto rssi = linkaiders::TX_POWER - link.get_distance();

    for (auto &tx_i : this->tx) {
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

        interference.push_back(linkaiders::TX_POWER - link_i.get_distance());
    }

    std::random_device rd{};
    std::mt19937 gen{rd()};
    auto pep = reachi::radiomodel::pep(rssi, linkaiders::PACKET_SIZE, interference);
    std::bernoulli_distribution d(1.0 - pep);
    return d(gen);
}
