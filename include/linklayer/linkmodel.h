#ifndef LINKLAYER_LINKMODEL_H
#define LINKLAYER_LINKMODEL_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize the link model.
 *
 * Will return nullptr if initialization fails.
 *
 * @param num_nodes Amount of nodes in the network
 * @param nchans Amount of channels in the network
 * @param gpslog Filepath for a log of GPS coordinates for all nodes
 * @return The link model object
 */
void *initialize(int num_nodes, int nchans, const char *gpslog);

/**
 * Deinitialize the link model.
 * @param model The link model object
 */
void deinit(void *model);

/**
 * Checks whether two nodes are connected at a given time (based on distance).
 * @param model The link model object
 * @param x Node identifier
 * @param y Node identifier
 * @param timestamp Timestamp to check
 * @return True if the nodes are connected
 */
bool is_connected(void *model, int x, int y, double timestamp);

/**
 * Notify the link model that a node starts sending on a specific channel.
 * @param model The link model object
 * @param id Node identifier
 * @param chn Channel identifier
 * @param timestamp Timestamp to start sending
 * @param duration Duration to transmit in
 */
void begin_send(void *model, int id, int chn, double timestamp, double duration);

/**
 * Notify the link model that a node stops sending on a specific channel.
 * @param model The link model object
 * @param id Node identifier
 * @param chn Channel identifier
 * @param timestamp Timestamp to stop sending
 */
void end_send(void *model, int id, int chn, double timestamp);

/**
 * Notify the link model that a node starts listening on a specific channel.
 * @param model The link model object
 * @param id Node identifier
 * @param chn Channel identifier
 * @param timestamp Timestamp to start listening
 * @param duration Duration to listen in
 */
void begin_listen(void *model, int id, int chn, double timestamp, double duration);

/**
 * Returns node identifier of the other node if transmission was completed before timestamp.
 * @param model The link model object
 * @param id Node identifier
 * @param chn Channel identifier
 * @param timestamp Timestamp to check for completion
 * @return Node identifier of the other node.
 */
int status(void *model, int id, int chn, double timestamp);

/**
 * Notify the link model that a node stops listening on a specific channel.
 * @param model The link model object
 * @param id Node identifier
 * @param chn Channel identifier
 * @param timestamp Timestamp to stop listening
 * @return Node identifier of the other node.
 */
int end_listen(void *model, int id, int chn, double timestamp);

/**
 * Get an array of node identifiers of all nodes alive at a given timestamp.
 *
 * The returned array should be free'd by caller.
 *
 * @param model The link model object
 * @param timestamp Timestamp to get alive nodes
 * @param node_count Amount of node identifiers returned
 * @return Array containing node identifiers
 */
int *alive_nodes(void *model, double timestamp, int *node_count);

#ifdef __cplusplus
}
#endif

#endif /* LINKLAYER_LINKMODEL_H */