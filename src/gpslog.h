#ifndef LINKAIDERS_GPSLOG_H
#define LINKAIDERS_GPSLOG_H

#include <unordered_map>

#include <reachi/node.h>
#include "model.h"

linkaiders::NodeMap parse_gpsfile(const char *gpslog);


#endif /* LINKAIDERS_GPSLOG_H */
