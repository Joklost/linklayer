#include "link.h"

linklayer::Link::Link(unsigned long long id, linklayer::Node &n1, linklayer::Node &n2) {
    this->id = id;
    this->nodes = std::make_pair(n1, n2);
}

bool linklayer::Link::operator==(const linklayer::Link &rhs) const {
    return id == rhs.id;
}

bool linklayer::Link::operator!=(const linklayer::Link &rhs) const {
    return !(rhs == *this);
}
