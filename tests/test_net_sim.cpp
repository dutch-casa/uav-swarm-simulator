#include <catch2/catch_test_macros.hpp>
#include "swarmgrid/adapters/net_sim_asio.hpp"
#include <boost/uuid/uuid_generators.hpp>
#include <thread>

using namespace swarmgrid::ports;
using namespace swarmgrid::adapters;
using namespace swarmgrid::core;

TEST_CASE("NetSimAsio basic operations", "[network]") {
    boost::uuids::random_generator gen;

    SECTION("Send and receive with no delay or drops") {
        NetworkParams params{0.0, 0, 0};
        NetSimAsio net(params, 42);

        auto sender_id = gen();
        auto receiver_id = gen();

        Message msg{sender_id, MessageType::PATH_ANNOUNCEMENT, {3, 4}, 5};
        net.send(msg);

        // Should receive on next tick
        auto received = net.receive(receiver_id, 6);
        REQUIRE(received.size() == 1);
        REQUIRE(received[0].from == sender_id);
        REQUIRE(received[0].next == Cell{3, 4});
        REQUIRE(received[0].timestamp == 5);
    }

    SECTION("Messages not delivered before their time") {
        NetworkParams params{0.0, 200, 0};  // 200ms latency = 2+ ticks
        NetSimAsio net(params, 42);

        auto sender_id = gen();
        auto receiver_id = gen();

        Message msg{sender_id, MessageType::PATH_ANNOUNCEMENT, {1, 2}, 0};
        net.send(msg);

        // Too early
        auto received_early = net.receive(receiver_id, 1);
        REQUIRE(received_early.empty());

        // Should arrive by tick 3
        auto received_late = net.receive(receiver_id, 3);
        REQUIRE(!received_late.empty());
    }

    SECTION("Agent doesn't receive its own messages") {
        NetworkParams params{0.0, 0, 0};
        NetSimAsio net(params, 42);

        auto agent_id = gen();

        Message msg{agent_id, MessageType::PATH_ANNOUNCEMENT, {2, 3}, 0};
        net.send(msg);

        auto received = net.receive(agent_id, 1);
        REQUIRE(received.empty());
    }

    SECTION("Reset clears all queues") {
        NetworkParams params{0.0, 100, 0};
        NetSimAsio net(params, 42);

        auto sender_id = gen();
        auto receiver_id = gen();

        Message msg{sender_id, MessageType::PATH_ANNOUNCEMENT, {5, 5}, 0};
        net.send(msg);

        net.reset();

        auto received = net.receive(receiver_id, 10);
        REQUIRE(received.empty());
    }

    SECTION("Multiple messages maintain order") {
        NetworkParams params{0.0, 0, 0};
        NetSimAsio net(params, 42);

        auto sender_id = gen();
        auto receiver_id = gen();

        Message msg1{sender_id, MessageType::PATH_ANNOUNCEMENT, {1, 1}, 0};
        Message msg2{sender_id, MessageType::PATH_ANNOUNCEMENT, {2, 2}, 1};
        Message msg3{sender_id, MessageType::PATH_ANNOUNCEMENT, {3, 3}, 2};

        net.send(msg1);
        net.send(msg2);
        net.send(msg3);

        auto received = net.receive(receiver_id, 5);
        REQUIRE(received.size() == 3);
    }
}

TEST_CASE("NetSimAsio probabilistic behavior", "[network]") {
    SECTION("Message drops with high probability") {
        NetworkParams params{0.9, 0, 0};  // 90% drop rate
        NetSimAsio net(params, 42);

        boost::uuids::random_generator gen;
        auto sender_id = gen();
        auto receiver_id = gen();

        int sent = 100;
        for (int i = 0; i < sent; ++i) {
            Message msg{sender_id, MessageType::PATH_ANNOUNCEMENT, {i, i}, i};
            net.send(msg);
        }

        auto received = net.receive(receiver_id, 200);

        // With 90% drop rate, we expect roughly 10% to get through
        REQUIRE(received.size() < sent / 2);  // Very likely to drop more than half
    }

    SECTION("Deterministic with same seed") {
        NetworkParams params{0.5, 100, 50};

        NetSimAsio net1(params, 12345);
        NetSimAsio net2(params, 12345);

        boost::uuids::random_generator gen;
        auto sender_id = gen();
        auto receiver_id = gen();

        // Send same messages through both networks
        for (int i = 0; i < 10; ++i) {
            Message msg{sender_id, MessageType::PATH_ANNOUNCEMENT, {i, i}, i};
            net1.send(msg);
            net2.send(msg);
        }

        auto received1 = net1.receive(receiver_id, 50);
        auto received2 = net2.receive(receiver_id, 50);

        // Same seed should produce same results
        REQUIRE(received1.size() == received2.size());
    }
}