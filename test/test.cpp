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

TEST_CASE("send/listen single channel no interference", "[linklayer/linkmodel]") {
    auto *model = TestModel::get_instance()->get_model();

    REQUIRE(is_connected(model, 17, 42, 3960000));
    REQUIRE_FALSE(is_connected(model, 17, 64, 3960000));
    begin_send(model, 17, 0, 3960000, 10);
    begin_send(model, 64, 0, 3960000, 20);
    begin_listen(model, 42, 0, 3960000, 10);
    REQUIRE(status(model, 42, 0, 3960005) == -1);
    REQUIRE(end_listen(model, 42, 0, 3960010) == 17);
}


TEST_CASE("send/listen multiple channels no interference", "[linklayer/linkmodel]") {
    auto *model = TestModel::get_instance()->get_model();

    REQUIRE(is_connected(model, 17, 49, 3960000));
    REQUIRE(is_connected(model, 42, 49, 3960000));
    begin_send(model, 17, 1, 3960000, 15);
    begin_send(model, 42, 0, 3960005, 20);
    begin_listen(model, 49, 1, 3960000, 40);
    /* Interference as both 17 and 42 transmits at the same time. */
    REQUIRE(status(model, 49, 1, 3960020) == 17);
    REQUIRE(end_listen(model, 49, 1, 3960020) == 17);
}

TEST_CASE("send/listen multiple channels interference", "[linklayer/linkmodel]") {
    auto *model = TestModel::get_instance()->get_model();

    REQUIRE(is_connected(model, 17, 49, 3960000));
    REQUIRE(is_connected(model, 42, 49, 3960000));
    begin_send(model, 17, 0, 3960000, 15);
    begin_send(model, 17, 1, 3960000, 15);
    begin_send(model, 42, 1, 3960005, 20);
    begin_listen(model, 49, 0, 3960000, 40);
    begin_listen(model, 49, 1, 3960000, 40);
    /* Interference as both 17 and 42 transmits at the same time. */
    REQUIRE(status(model, 49, 1, 3960025) == -1);
    REQUIRE(end_listen(model, 49, 1, 3960025) == -1);
    /* No interference as only 17 transmits on channel 0. */
    REQUIRE(status(model, 49, 0, 3960020) == 17);
    REQUIRE(end_listen(model, 49, 0, 3960020) == 17);
}



TEST_CASE("Send/Listen on channels", "[linklayer/linkmodel]") {
    auto *model = TestModel::get_instance()->get_model();

    /* 17 <-> 42 @ 3960000 */
    /* 17 <-> 49 @ 3960000 */
    /* 42 <-> 49 @ 3960000 */


    REQUIRE(is_connected(model, 17, 49, 3960000));
    REQUIRE(is_connected(model, 42, 49, 3960000));
    begin_send(model, 17, 1, 3960000, 15);
    begin_send(model, 42, 1, 3960005, 20);
    begin_listen(model, 49, 1, 3960005, 40);
    /* Interference as both 17 and 42 transmits at the same time. */
    REQUIRE(status(model, 49, 1, 3960020) == -1);
    REQUIRE(end_listen(model, 49, 1, 3960020) == -1);

    REQUIRE(is_connected(model, 17, 49, 3960000));
    begin_send(model, 17, 1, 3960025, 20);
    begin_listen(model, 49, 1, 3960025, 30);
    end_send(model, 42, 1, 3960019);
    REQUIRE(status(model, 49, 1, 3960050) == 17);
    REQUIRE(end_listen(model, 49, 1, 3960055) == 17);

    deinit(model);
}

TEST_CASE("alive_nodes()", "[linklayer/linkmodel]") {
    auto *model = TestModel::get_instance()->get_model();

    int node_count;
    int *nodes;

    nodes = alive_nodes(model, 20000, &node_count);
    delete[] nodes;
    REQUIRE(node_count == 0);

    nodes = alive_nodes(model, 3960000, &node_count);
    delete[] nodes;
    REQUIRE(node_count == 24);

    nodes = alive_nodes(model, 5240000, &node_count);
    delete[] nodes;
    REQUIRE(node_count == 3);

    deinit(model);
}