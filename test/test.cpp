#include <iostream>
#include <string>
#include <cstdio>
#include <unistd.h>

#include <catch2/catch.hpp>

#include <linklayer/linkmodel.h>
#include "../src/model.h"

void *get_test_model() {
    char logpath[] = "gpslog_rssi.txt";
    return initialize(2, logpath);
}

class TestModel {
private:
    void *model;
    linklayer::LinkModel *linkmodel;

    TestModel() {
        std::cout << "Constructing test model... " << std::flush;
        this->model = get_test_model();
        this->linkmodel = static_cast<linklayer::LinkModel *>(this->model);
        std::cout << "OK\n\n" << std::flush;
    }

    virtual ~TestModel() {
        std::cout << "Freeing test model... " << std::flush;
        deinit(this->model);
        std::cout << "OK\n" << std::flush;
    }

public:
    static TestModel *get_instance() {
        static TestModel instance;

        return &instance;
    }

    void *get_model() {
        return static_cast<void *>(new linklayer::LinkModel{*this->linkmodel});
    }
};


TEST_CASE("Initialize link model object", "[linklayer/linkmodel]") {
    auto *model = TestModel::get_instance()->get_model();
    REQUIRE(model);
    deinit(model);
    model = initialize(0, nullptr);
    REQUIRE_FALSE(model);
}

TEST_CASE("Test link reciprocation", "[linklayer/linkmodel]") {
    auto *model = TestModel::get_instance()->get_model();
    auto *linkmodel = static_cast<linklayer::LinkModel *>(model);

    auto &nodes = linkmodel->node_map;

//    auto &topology = linkmodel->get_topology(3960000);
//    for (auto &link : topology.links) {
//        std::cout << link.nodes.first.id << " " << link.nodes.second.id << " " << link.rssi << std::endl;
//    }

    for (auto &topology : linkmodel->topologies) {
        linkmodel->get_topology(topology.first);
        std::cout << topology.first << " " << topology.second.links.size() << std::endl;
    }
}

//TEST_CASE("Topology generation", "[linklayer/linkmodel]") {
//    auto *model = TestModel::get_instance()->get_model();
//    auto *linkmodel = static_cast<linklayer::LinkModel *>(model);
//
//    auto &topologies = linkmodel->topologies;
//
//    REQUIRE(topologies.size() == 494);
//
//    for (const auto &topology : topologies) {
//        auto time = topology.first;
//        auto &links = linkmodel->get_topology(time).links;
//
//        for (const auto &link : links) {
//            auto &nodes = link.nodes;
//
//            CHECK_FALSE(nodes.first == nodes.second);
//            CHECK(nodes.first.location.get_time() <= time);
//            CHECK(nodes.second.location.get_time() <= time);
//            CHECK_FALSE(link.pathloss == Approx(0.0));
//        }
//    }
//
//    deinit(model);
//}


//TEST_CASE("Topology connectedness", "[linklayer/linkmodel]") {
//    auto *model = TestModel::get_instance()->get_model();
//    auto *linkmodel = static_cast<linklayer::LinkModel *>(model);
//
//    for (const auto &topology : linkmodel->topologies) {
//        auto time = topology.first;
//        auto &links = linkmodel->get_topology(time).links;
//
//        for (const auto &link : links) {
//            auto &node1 = link.nodes.first;
//            auto &node2 = link.nodes.second;
//
//            if ((linklayer::TX_POWER - link.pathloss) < linklayer::DISTANCE_THRESHOLD) {
//                CHECK_FALSE(is_connected(model, node1.id, node2.id, time));
//            } else {
//                CHECK(is_connected(model, node1.id, node2.id, time));
//            }
//        }
//    }
//
//    deinit(model);
//}

//TEST_CASE("Broadcast/Listen on channels", "[linklayer/linkmodel]") {
//    auto *model = TestModel::get_instance()->get_model();
//
//    begin_send(model, 40, 0, 0.0, 10.0);
//    begin_listen(model, 10, 0, 0.0, 10.0);
//    REQUIRE(status(model, 10, 0, 7.0) == -1);
//    REQUIRE(end_listen(model, 10, 0, 10.0) == 40);
//
//
//    deinit(model);
//}

//TEST_CASE("Generate topology for timestamp", "[linklayer/linkmodel]") {
//    auto *model = TestModel::get_instance()->get_model();
//
//    int node_count;
//    int *nodes;
//    nodes = alive_nodes(model, 20000, &node_count);
//    delete[] nodes;
//    REQUIRE(node_count == 32); /* 32 out of 33 nodes present at 20 seconds. */
//    nodes = alive_nodes(model, 5240000, &node_count);
//    REQUIRE(node_count == 2);
//    delete[] nodes;
//
//    deinit(model);
//}