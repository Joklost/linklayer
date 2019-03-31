#ifndef LINKLAYER_LOCATION_H
#define LINKLAYER_LOCATION_H

#include <unordered_map>

#include <geo/location.h>

namespace linklayer {

    struct Location : public geo::Location {
        Location() = default;

        Location(double time, double latitude, double longitude) : geo::Location(time, latitude, longitude) {}

        std::unordered_map<unsigned long, double> connections{};
    };

}

#endif //LINKLAYER_LOCATION_H
