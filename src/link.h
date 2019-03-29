#ifndef LINKLAYER_LINK_H
#define LINKLAYER_LINK_H

#include <utility>
#include "node.h"

namespace linklayer {

    struct Link {

        Link() = default;
        Link(unsigned long long id, linklayer::Node &n1, linklayer::Node &n2);

        bool operator==(const Link &rhs) const;

        bool operator!=(const Link &rhs) const;

        unsigned long long id{};
        std::pair<linklayer::Node, linklayer::Node> nodes;

        double pathloss{};

    };

}


#endif //LINKLAYER_LINK_H
