#include <iostream>
#include <algorithm>
#include <iterator>
#include <set>

#include <lm/linkmodel.h>

#include <sims/node.h>
#include <common/iters.h>

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
    lm::NodeMap node_map;
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

    /* Return model as void pointer. */
    auto *lm = new lm::LinkModel{nchans, node_map};
    return static_cast<void *>(lm);
}

void deinit(void *model) {
    if (model == nullptr) {
        return;
    }

    auto *lm = static_cast<lm::LinkModel *>(model);
    delete lm;
}

bool is_connected(void *model, int x, int y, double timestamp) {
    auto *lm = static_cast<lm::LinkModel *>(model);

    auto &link = lm->get_link(x, y, timestamp);
    if (link.get_id() == 0ul) {
        return false;
    }

    return lm::TX_POWER - link.get_distance() > lm::DISTANCE_THRESHOLD;
}

void begin_send(void *model, int id, int chn, double timestamp, double duration) {
    auto *lm = static_cast<lm::LinkModel *>(model);
    lm->tx[chn].emplace_back(lm::Transmit, id, chn, timestamp, timestamp + duration);
}

void end_send(void *model, int id, int chn, double timestamp) {
    auto *lm = static_cast<lm::LinkModel *>(model);
    auto it = std::find(lm->tx[chn].begin(), lm->tx[chn].end(), lm::Action{lm::Transmit, id, chn});
    if (it != lm->tx[chn].end()) {
        auto &tx = *it;
        tx.end = timestamp;
    }
}

void begin_listen(void *model, int id, int chn, double timestamp, double duration) {
    auto *lm = static_cast<lm::LinkModel *>(model);
    lm->rx[chn].emplace_back(lm::Listen, id, chn, timestamp, timestamp + duration);
}

static int process_listen(lm::LinkModel *lm, int chn, lm::Action &rx) {
    std::random_device rd{};
    std::mt19937 gen{rd()};

    std::vector<lm::Action> tx_list{};
    for (auto &tx : lm->tx[chn]) {
        if (tx.is_within(rx)) {
            tx_list.push_back(tx);
        }
    }

    if (tx_list.size() == 1) {
        /* Only one transmitting node. */
        auto pep = lm->should_receive(tx_list.back(), rx, tx_list);
        std::bernoulli_distribution d{1.0 - pep};

        if (d(gen)) {
            return tx_list.back().id;
        }
    } else {
        std::vector<std::pair<unsigned long, double>> peps(tx_list.size());
        common::enumerate(tx_list.begin(), tx_list.end(), 0, [&lm, &rx, &tx_list, &peps](int c, lm::Action tx) {
            peps[c] = std::make_pair(tx.id, lm->should_receive(tx, rx, tx_list));
        });

        auto pep = std::min_element(peps.begin(), peps.end());
        if (pep == peps.end()) {
            return lm::LM_ERROR;
        }

        std::bernoulli_distribution d{1.0 - (*pep).second};
        if (d(gen)) {
            return (*pep).first;
        }
    }

    return lm::LM_ERROR;
}

int status(void *model, int id, int chn, double timestamp) {
    auto *lm = static_cast<lm::LinkModel *>(model);

    if (lm->tx[chn].empty()) {
        return lm::LM_ERROR;
    }

    auto it = std::find(lm->rx[chn].begin(), lm->rx[chn].end(), lm::Action{lm::Listen, id, chn});
    if (it == lm->rx[chn].end()) {
        return lm::LM_ERROR;
    }

    auto &rx = *it;
    auto original_time = rx.end;
    rx.end = timestamp;

    auto result = process_listen(lm, chn, rx);
    rx.end = original_time;
    return result;
}

int end_listen(void *model, int id, int chn, double timestamp) {
    auto *lm = static_cast<lm::LinkModel *>(model);

    if (lm->tx[chn].empty()) {
        return lm::LM_ERROR;
    }

    auto it = std::find(lm->rx[chn].begin(), lm->rx[chn].end(), lm::Action{lm::Listen, id, chn});
    if (it == lm->rx[chn].end()) {
        return lm::LM_ERROR;
    }

    auto &rx = *it;
    rx.end = timestamp;

    return process_listen(lm, chn, rx);
}

int *alive_nodes(void *model, double timestamp, int *node_count) {
    auto *lm = static_cast<lm::LinkModel *>(model);

    auto &topology = lm->get_topology(timestamp);
    std::set<unsigned long> node_ids{};

    for (auto &link : topology.links) {
        node_ids.emplace(link.get_nodes().first.get_id());
        node_ids.emplace(link.get_nodes().second.get_id());
    }

    *node_count = static_cast<int>(node_ids.size());
    auto nodes = new int[node_ids.size()];
    std::copy(node_ids.begin(), node_ids.end(), nodes);

    return nodes;
}

#ifdef __cplusplus
}

#endif