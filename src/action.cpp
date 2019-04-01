#include "action.h"

bool linklayer::Action::is_within(const Action &action) const {
    return this->start >= action.start && this->end <= action.end;
}

bool linklayer::Action::operator==(const linklayer::Action &rhs) const {
    return state == rhs.state &&
           id == rhs.id &&
           chn == rhs.chn;
}

bool linklayer::Action::operator!=(const linklayer::Action &rhs) const {
    return !(rhs == *this);
}
