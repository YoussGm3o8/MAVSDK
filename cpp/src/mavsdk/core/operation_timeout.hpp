#pragma once

#include "mavsdk_time.hpp"

#include <algorithm>
#include <chrono>
#include <optional>

namespace mavsdk {

class OperationTimeout {
public:
    OperationTimeout() = default;

    OperationTimeout(std::chrono::milliseconds timeout, SteadyTimePoint now) :
        _deadline(now + timeout)
    {}

    [[nodiscard]] bool is_expired(SteadyTimePoint now) const
    {
        return _deadline.has_value() && now >= *_deadline;
    }

    [[nodiscard]] std::optional<double> remaining_s(SteadyTimePoint now) const
    {
        if (!_deadline) {
            return std::nullopt;
        }
        const auto remaining = std::chrono::duration<double>(*_deadline - now).count();
        return (std::max)(0.0, remaining);
    }

    [[nodiscard]] double
    attempt_timeout_s(SteadyTimePoint now, unsigned retries_to_do, double default_timeout_s) const
    {
        if (!_deadline) {
            return default_timeout_s;
        }

        const auto remaining = *remaining_s(now);
        const auto attempts_left = static_cast<double>(retries_to_do + 1U);
        const std::chrono::duration<double> interval{(std::max)(0.0, remaining / attempts_left)};
        // The timer stores milliseconds; rounding down can retry an acknowledged command just
        // before its deadline.
        return std::chrono::duration<double>(std::chrono::ceil<std::chrono::milliseconds>(interval))
            .count();
    }

private:
    std::optional<SteadyTimePoint> _deadline{};
};

} // namespace mavsdk
