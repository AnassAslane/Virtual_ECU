#include "safety.hpp"
#include <gtest/gtest.h>

TEST(SafetyTest, StartsInInit)
{
    vecu::SafetyManager safety;
    EXPECT_EQ(safety.get_state(), vecu::SafetyState::INIT);
}

TEST(SafetyTest, SensorTimeoutLeadsToSafe)
{
    vecu::SafetyManager safety;
    safety.set_fault(vecu::FaultCode::SENSOR_TIMEOUT);
    safety.update_state();

    EXPECT_EQ(safety.get_state(), vecu::SafetyState::SAFE);
}
