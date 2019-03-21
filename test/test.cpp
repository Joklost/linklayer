#include <iostream>
#include <string>
#include <cstdio>
#include <unistd.h>

#include <catch2/catch.hpp>

#include <lm/linkmodel.h>
#include "../src/model.h"

void *get_test_model() {
    char logpath[] = "gpslog.txt";
    return initialize(33, 2, logpath);
}

class TestModel {
private:
    void *model;
    linkaiders::LinkModel *linkmodel;

    TestModel() {
        std::cout << "Constructing test model... " << std::flush;
        this->model = get_test_model();
        this->linkmodel = static_cast<linkaiders::LinkModel *>(this->model);
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
        return static_cast<void *>(new linkaiders::LinkModel{*this->linkmodel});
    }
};


TEST_CASE("Initialize link model object", "[lm/linkmodel]") {
    auto *model = TestModel::get_instance()->get_model();
    REQUIRE(model);
    deinit(model);
    model = initialize(0, 0, nullptr);
    REQUIRE_FALSE(model);
}

TEST_CASE("Topology generation", "[lm/linkmodel]") {
    auto *model = TestModel::get_instance()->get_model();
    auto *linkmodel = static_cast<linkaiders::LinkModel *>(model);

    auto &topologies = linkmodel->topologies;

    REQUIRE(topologies.size() == 494);

    for (const auto &topology : topologies) {
        auto time = topology.first;
        auto &links = topology.second.links;

        for (const auto &link : links) {
            auto &nodes = link.get_nodes();

            CHECK_FALSE(nodes.first == nodes.second);
            CHECK(nodes.first.current_location.get_time() <= time);
            CHECK(nodes.second.current_location.get_time() <= time);
            CHECK_FALSE(link.distance == Approx(0.0));
        }
    }

    deinit(model);
}


TEST_CASE("Topology connectedness", "[lm/linkmodel]") {
    auto *model = TestModel::get_instance()->get_model();
    auto *linkmodel = static_cast<linkaiders::LinkModel *>(model);

    for (const auto &topology : linkmodel->topologies) {
        auto time = topology.first;
        auto &links = topology.second.links;

        for (const auto &link : links) {
            auto &node1 = link.get_nodes().first;
            auto &node2 = link.get_nodes().second;

            if ((linkaiders::TX_POWER - link.distance) < linkaiders::DISTANCE_THRESHOLD) {
                CHECK_FALSE(is_connected(model, node1.get_id(), node2.get_id(), time));
            } else {
                CHECK(is_connected(model, node1.get_id(), node2.get_id(), time));
            }
        }
    }

    deinit(model);
}

TEST_CASE("Broadcast/Listen on channels", "[lm/linkmodel]") {
//    void *model = get_test_model();
//    begin_send(model, 40, 0, 1510715240, 10);
//    begin_listen(model, 10, 0, 1510715240, 10);
//    begin_listen(model, 11, 0, 1510715240, 10);
//    end_send(model, 40, 0, 1510715245);
//
//    REQUIRE(status(model, 10, 0, 1510715247) == 40);
//    REQUIRE(end_listen(model, 10, 0, 1510715250) == 40);
//    REQUIRE(end_listen(model, 41, 0, 1510715250) == -1);

}
