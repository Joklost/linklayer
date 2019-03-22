
#ifndef LINKAIDERS_MODEL_H
#define LINKAIDERS_MODEL_H

#include <utility>
#include <random>
#include <map>
#include <unordered_map>

#include <sims/link.h>

namespace linkaiders {
    const int LM_SUCCESS = 0;
    const int LM_ERROR = -1;

    const unsigned long PACKET_SIZE = 20;
    const double DISTANCE_THRESHOLD = -110.0;
    const double TX_POWER = 26.0;

    enum State {
        Idle,
        Listen,
        Transmit,
    };

    /* Model */
    struct Action {
        State state{};
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
        std::vector<sims::Link> links{};
    };

    using NodeMap = std::unordered_map<unsigned long, sims::Node>;
    using TopologyMap = std::map<double, Topology>;

    struct LinkModel {
        NodeMap nodes{};
        TopologyMap topologies{};

        std::vector<Action> tx{};
        std::vector<std::vector<Action>> rx{};

        LinkModel(int nchans, NodeMap node_map, TopologyMap topology_map) : tx(nchans), rx(nchans),
                                                                            nodes(std::move(node_map)),
                                                                            topologies(std::move(topology_map)) {};

        const sims::Link get_link(int x, int y, double timestamp) const;

        bool should_receive(const Action &t, const Action &r);
    };
}


#endif /* LINKAIDERS_MODEL_H */
