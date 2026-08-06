#include "litearm/exceptions.hpp"
#include <unordered_set>

namespace litearm {

namespace {

const std::unordered_set<std::string> SAFETY_VIOLATION_TYPES = {
    "SafetyViolationError",
    "FeedbackTimeoutError",
    "FollowingError",
    "MotionTimeoutError",
    "MotorFaultError",
    "ArmFault",
    "WatchdogError",
};

} // anonymous namespace

bool is_safety_violation_type(const std::string& type_name) {
    return SAFETY_VIOLATION_TYPES.count(type_name) > 0;
}

[[noreturn]] void throw_exception(
    const std::string& type_name,
    const std::string& message,
    std::unordered_map<std::string, std::string> details)
{
    // Safety violation subclasses — throw with details
    if (type_name == "FeedbackTimeoutError")
        throw FeedbackTimeoutError(message, std::move(details));
    if (type_name == "FollowingError")
        throw FollowingError(message, std::move(details));
    if (type_name == "MotionTimeoutError")
        throw MotionTimeoutError(message, std::move(details));
    if (type_name == "MotorFaultError")
        throw MotorFaultError(message, std::move(details));
    if (type_name == "ArmFault")
        throw ArmFault(message, std::move(details));
    if (type_name == "WatchdogError")
        throw WatchdogError(message, std::move(details));
    if (type_name == "SafetyViolationError")
        throw SafetyViolationError(message, std::move(details));

    // CartesianPlanError
    if (type_name == "CartesianPlanError")
        throw CartesianPlanError(message);

    // Other known types
    if (type_name == "ConfigurationError")
        throw ConfigurationError(message);
    if (type_name == "StateTransitionError")
        throw StateTransitionError(message);
    if (type_name == "NotConnectedError")
        throw NotConnectedError(message);
    if (type_name == "InvalidCommandError")
        throw InvalidCommandError(message);
    if (type_name == "TransportError")
        throw TransportError(message);
    if (type_name == "CanModeMismatch")
        throw CanModeMismatch(message);
    if (type_name == "CanMotorNotRegistered")
        throw CanMotorNotRegistered(message);
    if (type_name == "CanWriteTimeout")
        throw CanWriteTimeout(message);
    if (type_name == "MotionCancelled")
        throw MotionCancelled(message);

    // Fallback
    throw LiteArmError(message);
}

} // namespace litearm
