#ifndef LINKLAYER_NODE_H
#define LINKLAYER_NODE_H

#include <vector>

#include "location.h"

namespace linklayer {

    struct Node {
        Node() = default;

        Node(unsigned long id, linklayer::Location);

        bool operator==(const Node &rhs) const;

        bool operator!=(const Node &rhs) const;

        unsigned long id{};
        linklayer::Location location{};
        std::vector<linklayer::Location> location_history{};
    };

}


#endif //LINKLAYER_NODE_H
