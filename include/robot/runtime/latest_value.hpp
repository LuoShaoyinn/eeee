#pragma once

#include <mutex>
#include <optional>
#include <utility>

namespace robot {

// A camera/control consumer needs the newest sample, never an old backlog.
template <typename T>
class LatestValue {
public:
    void publish(T value) {
        std::scoped_lock lock(mutex_);
        value_ = std::move(value);
    }

    std::optional<T> take() {
        std::scoped_lock lock(mutex_);
        auto result = std::move(value_);
        value_.reset();
        return result;
    }

private:
    std::mutex mutex_;
    std::optional<T> value_;
};

}  // namespace robot
