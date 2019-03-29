#ifndef LINKLAYER_NODE_H
#define LINKLAYER_NODE_H

#include <vector>

#include <geo/location.h>

namespace linklayer {

    struct Node {
        Node() = default;

        Node(unsigned long id, geo::Location);

        bool operator==(const Node &rhs) const;

        bool operator!=(const Node &rhs) const;

        unsigned long id{};
        geo::Location location{};

        std::vector<geo::Location> location_history{};
    };

}


#endif //LINKLAYER_NODE_H
