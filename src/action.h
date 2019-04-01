#ifndef LINKLAYER_ACTION_H
#define LINKLAYER_ACTION_H

namespace linklayer {

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

        Action() = default;

        Action(linklayer::State state, int id, int chn) : state(state), id(id), chn(chn) {}

        Action(linklayer::State state, int id, int chn, double start) : state(state), id(id),
                                                                        chn(chn), start(start) {}

        Action(linklayer::State state, int id, int chn, double start, double end) : state(state), id(id), chn(chn),
                                                                                    start(start), end(end) {}
    };

}



#endif //LINKLAYER_ACTION_H
