
#ifndef LINKAIDERS_MODEL_H
#define LINKAIDERS_MODEL_H

#include <utility>
#include <random>
#include <map>
#include <unordered_map>

#include <common/equality.h>
#include <sims/link.h>

namespace lm {
    const int LM_ERROR = -1;

    const unsigned long PACKET_SIZE = 20;
    const double DISTANCE_THRESHOLD = -110.0;
    const double TX_POWER = 26.0;
    const double TIME_GAP = 20000.0;

    enum State {
        Idle,
        Listen,
        Transmit,
    };

    struct Action {
        State state{Idle};
        int id{};
        int chn{};
        double start{};
        double end{};

        bool is_within(const Action &action) const;

        bool operator==(const Action &rhs) const;

        bool operator!=(const Action &rhs) const;

        Action();

        Action(State type, int id, int chn);

        Action(State state, int id, int chn, double start);

        Action(State state, int id, int chn, double start, double end);
    };

    struct Topology {
        double timestamp{};
        std::vector<sims::Link> links{};
    };

    using NodeMap = std::unordered_map<unsigned long, sims::Node>;
    using NodeList = std::vector<sims::Node>;
    using TopologyMap = std::map<double, Topology, common::is_less<double>>;

    struct LinkModel {
        NodeMap node_map{};
        TopologyMap topologies{};
        NodeList node_list{};

        std::vector<std::vector<Action>> tx{};
        std::vector<std::vector<Action>> rx{};

        LinkModel(int nchans, NodeMap n_map);

        const sims::Link get_link(int x, int y, double timestamp);

        double should_receive(const Action &t, const Action &r, const std::vector<Action> &tx_list);

        Topology &get_topology(double timestamp);
    };
}


#endif /* LINKAIDERS_MODEL_H */
