#include "diagnostics.hpp"
#include <gtest/gtest.h>

TEST(DiagnosticsTest, ReadDtcRequestReturnsPositiveResponse)
{
    vecu::Diagnostics diagnostics;
    diagnostics.record_fault(vecu::FaultCode::OVERSPEED);

    const auto response = diagnostics.process_request({0x19, 0x02});

    ASSERT_GE(response.size(), 2U);
    EXPECT_EQ(response[0], 0x59);
    EXPECT_EQ(response[1], 0x02);
}

TEST(DiagnosticsTest, ClearDtc)
{
    vecu::Diagnostics diagnostics;
    diagnostics.record_fault(vecu::FaultCode::OVERSPEED);
    diagnostics.process_request({0x14});

    EXPECT_TRUE(diagnostics.get_dtcs().empty());
}
