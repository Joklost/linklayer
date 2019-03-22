#include <iostream>
#include <algorithm>
#include <iterator>

#include <sims/math.h>
#include <sims/node.h>

#include <lm/linkmodel.h>
#include <common/helpers.h>

#include "model.h"
#include "gpslog.h"

#ifdef __cplusplus
extern "C" {
#endif

void *initialize(int num_nodes, int nchans, const char *gpslog) {
    if (!gpslog || num_nodes <= 0 || nchans <= 0) {
        return nullptr;
    }

    /* Parse GPS log. */
    linkaiders::NodeMap node_map;
    try {
        node_map = parse_gpsfile(gpslog);
    } catch (const std::exception &e) {
        std::cerr << e.what() << std::endl;
        return nullptr;
    }

    if (node_map.empty()) {
        std::cerr << "failed to parse gpslog file" << std::endl;
        return nullptr;
    }

    /* Generate topologies. */
    std::vector<sims::Node> node_list{};
    node_list.reserve(node_map.size());
    for (auto &item : node_map) {
        auto node = item.second;
        node_list.push_back(node);
    }

    linkaiders::TopologyMap topologies{};

    for (auto &node : node_list) {
        for (auto &location : node.location_history) {
            if (topologies.find(location.get_time()) == topologies.end()) {
                topologies[location.get_time()] = {};
            }
        }
    }

    for (auto &topology : topologies) {
        const auto time = topology.first;
        auto &links = topology.second.links;

        for (unsigned long i = 0; i < node_list.size(); ++i) {
            auto node1 = node_list[i]; /* Copy */

            for (auto &location : node1.location_history) {
                if (location.get_time() <= time || common::is_equal(location.get_time(), time)) {
                    node1.current_location = location;
                }
            }

            for (unsigned long j = i + 1; j < node_list.size(); ++j) {
                auto node2 = node_list[j]; /* Copy */

                for (auto &location : node2.location_history) {
                    if (location.get_time() <= time) {
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

    /* Return model as void pointer. */
    auto *lm = new linkaiders::LinkModel{nchans, node_map, topologies};
    return static_cast<void *>(lm);
}

void deinit(void *model) {
    if (model == nullptr) {
        return;
    }

    auto *lm = static_cast<linkaiders::LinkModel *>(model);
    delete lm;
}

bool is_connected(void *model, int x, int y, double timestamp) {
    auto *lm = static_cast<linkaiders::LinkModel *>(model);

    auto &link = lm->get_link(x, y, timestamp);
    if (link.get_id() == 0ul) {
        return false;
    }

    return linkaiders::TX_POWER - link.get_distance() > linkaiders::DISTANCE_THRESHOLD;
}

void begin_send(void *model, int id, int chn, double timestamp, double duration) {
    auto *lm = static_cast<linkaiders::LinkModel *>(model);
    lm->tx[chn] = {linkaiders::Transmit, id, chn, timestamp, timestamp + duration};
}

void end_send(void *model, int id, int chn, double timestamp) {
    auto *lm = static_cast<linkaiders::LinkModel *>(model);
    lm->tx[chn].end = timestamp;
}

void begin_listen(void *model, int id, int chn, double timestamp, double duration) {
    auto *lm = static_cast<linkaiders::LinkModel *>(model);
    lm->rx[chn].emplace_back(linkaiders::Listen, id, chn, timestamp, timestamp + duration);
}

int status(void *model, int id, int chn, double timestamp) {
    auto *lm = static_cast<linkaiders::LinkModel *>(model);

    auto &tx = lm->tx[chn];
    if (tx.state == linkaiders::Idle) {
        return linkaiders::LM_ERROR;
    }

    auto it = std::find(lm->rx[chn].begin(), lm->rx[chn].end(), linkaiders::Action{linkaiders::Listen, id, chn});
    if (it == lm->rx[chn].end()) {
        return linkaiders::LM_ERROR;
    }

    auto &rx = *it;
    auto end_timestamp = rx.end;
    rx.end = timestamp;

    if (!tx.is_within(rx)) {
        return linkaiders::LM_ERROR;
    }

    if (lm->should_receive(tx, rx)) {
        return tx.id;
    }

    rx.end = end_timestamp;
    return linkaiders::LM_ERROR;
}

int end_listen(void *model, int id, int chn, double timestamp) {
    auto *lm = static_cast<linkaiders::LinkModel *>(model);

    auto &tx = lm->tx[chn];
    if (tx.state == linkaiders::Idle) {
        return linkaiders::LM_ERROR;
    }

    auto it = std::find(lm->rx[chn].begin(), lm->rx[chn].end(), linkaiders::Action{linkaiders::Listen, id, chn});
    if (it == lm->rx[chn].end()) {
        return linkaiders::LM_ERROR;
    }

    auto &rx = *it;
    rx.end = timestamp;
    if (!tx.is_within(rx)) {
        return linkaiders::LM_ERROR;
    }

    if (lm->should_receive(tx, rx)) {
        return tx.id;
    }

    return linkaiders::LM_ERROR;
}

#ifdef __cplusplus
}
#endif