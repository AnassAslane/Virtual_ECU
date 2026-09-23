#include "timeout_monitor.hpp"
#include <gtest/gtest.h>

TEST(TimeoutMonitorTest, UnknownIdIsTimeout)
{
    vecu::TimeoutMonitor monitor;
    EXPECT_EQ(monitor.status(0x999), vecu::TimeoutStatus::TIMEOUT);
}

TEST(TimeoutMonitorTest, ReceivedMessageIsHealthy)
{
    vecu::TimeoutMonitor monitor;
    monitor.frame_received(0x100);
    monitor.update();

    EXPECT_EQ(monitor.status(0x100), vecu::TimeoutStatus::OK);
}
