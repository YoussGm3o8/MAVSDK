#include "operation_timeout.hpp"

#include <chrono>

#include <gtest/gtest.h>

namespace mavsdk {

TEST(OperationTimeout, UsesDefaultAttemptTimeoutWithoutOperationBudget)
{
    const OperationTimeout timeout{};
    EXPECT_DOUBLE_EQ(timeout.attempt_timeout_s(SteadyTimePoint{}, 3, 0.5), 0.5);
}

TEST(OperationTimeout, DividesRemainingBudgetAcrossRetries)
{
    const auto start = SteadyTimePoint{};
    const OperationTimeout timeout{std::chrono::milliseconds(400), start};

    EXPECT_DOUBLE_EQ(timeout.attempt_timeout_s(start, 3, 0.5), 0.1);
    EXPECT_DOUBLE_EQ(
        timeout.attempt_timeout_s(start + std::chrono::milliseconds(100), 2, 0.5), 0.1);
}

TEST(OperationTimeout, IncludesQueueTimeAndExpiresAtDeadline)
{
    const auto start = SteadyTimePoint{};
    const OperationTimeout timeout{std::chrono::milliseconds(100), start};

    EXPECT_FALSE(timeout.is_expired(start + std::chrono::milliseconds(99)));
    EXPECT_TRUE(timeout.is_expired(start + std::chrono::milliseconds(100)));
    EXPECT_DOUBLE_EQ(*timeout.remaining_s(start + std::chrono::milliseconds(50)), 0.05);
    EXPECT_DOUBLE_EQ(
        timeout.attempt_timeout_s(start + std::chrono::milliseconds(120), 3, 0.5), 0.0);
}

} // namespace mavsdk
