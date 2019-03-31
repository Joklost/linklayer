#include "node.h"

linklayer::Node::Node(unsigned long id, linklayer::Location l) {
    this->id = id;
    this->location = l;
}

bool linklayer::Node::operator==(const linklayer::Node &rhs) const {
    return id == rhs.id;
}

bool linklayer::Node::operator!=(const linklayer::Node &rhs) const {
    return !(rhs == *this);
}
