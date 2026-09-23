#include "signal_manager.hpp"
#include <gtest/gtest.h>

TEST(SignalManagerTest, DecodesEngineFrame)
{
    vecu::SignalManager manager;
    vecu::CanFrame frame{};

    frame.id = 0x100;
    frame.dlc = 7;
    frame.data[0] = 0x90;
    frame.data[1] = 0x01;
    frame.data[2] = 0xE8;
    frame.data[3] = 0x03;
    frame.data[4] = 50;
    frame.data[5] = 90;
    frame.data[6] = 1;

    ASSERT_TRUE(manager.process_frame(frame));

    EXPECT_EQ(manager.get_signals().rpm, 400);
    EXPECT_FLOAT_EQ(manager.get_signals().speed_kmh, 100.0F);
    EXPECT_EQ(manager.get_signals().throttle_percent, 50);
    EXPECT_EQ(manager.get_signals().engine_temperature_c, 90);
    EXPECT_TRUE(manager.get_signals().engine_running);
}
