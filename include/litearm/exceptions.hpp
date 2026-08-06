#pragma once
/**
 * @file exceptions.hpp
 * @brief Structured exceptions for the litearm C++ client SDK.
 *
 * Mirrors pylitearm.errors and litearm-python exceptions — no cross-imports.
 */

#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>
#include <utility>

namespace litearm {

// ── Base exception ──────────────────────────────────────────────────────────

/// Base class for all litearm SDK runtime errors.
class LiteArmError : public std::runtime_error {
public:
    explicit LiteArmError(const std::string& msg) : std::runtime_error(msg) {}
};

// ── Configuration ───────────────────────────────────────────────────────────

/// Robot configuration is missing, inconsistent, or fails integrity checks.
class ConfigurationError : public LiteArmError {
public:
    explicit ConfigurationError(const std::string& msg) : LiteArmError(msg) {}
};

// ── State transitions ───────────────────────────────────────────────────────

/// An API operation is not valid in the current arm state.
class StateTransitionError : public LiteArmError {
public:
    explicit StateTransitionError(const std::string& msg) : LiteArmError(msg) {}
};

/// The requested operation requires a live hardware connection.
class NotConnectedError : public StateTransitionError {
public:
    explicit NotConnectedError(const std::string& msg) : StateTransitionError(msg) {}
};

// ── Invalid commands ────────────────────────────────────────────────────────

/// A target or control parameter is malformed or outside its safe range.
class InvalidCommandError : public LiteArmError {
public:
    explicit InvalidCommandError(const std::string& msg) : LiteArmError(msg) {}
};

/// Cartesian planning failed (e.g. IK singularity or path infeasible).
class CartesianPlanError : public InvalidCommandError {
public:
    explicit CartesianPlanError(const std::string& msg,
                                int index = -1)
        : InvalidCommandError(msg), index(index) {}

    int index;  ///< Index of the failing waypoint (-1 = unknown)
};

// ── Safety violations ───────────────────────────────────────────────────────

/// A planned or executing motion violated a configured safety rule.
class SafetyViolationError : public LiteArmError {
public:
    explicit SafetyViolationError(
        const std::string& msg,
        std::unordered_map<std::string, std::string> details = {})
        : LiteArmError(msg), details(std::move(details)) {}

    std::unordered_map<std::string, std::string> details;
};

/// One or more joints did not provide sufficiently fresh feedback.
class FeedbackTimeoutError : public SafetyViolationError {
public:
    explicit FeedbackTimeoutError(
        const std::string& msg,
        std::unordered_map<std::string, std::string> details = {})
        : SafetyViolationError(msg, std::move(details)) {}
};

/// Measured joint position departed too far from the commanded trajectory.
class FollowingError : public SafetyViolationError {
public:
    explicit FollowingError(
        const std::string& msg,
        std::unordered_map<std::string, std::string> details = {})
        : SafetyViolationError(msg, std::move(details)) {}
};

/// The robot did not physically settle at the requested target in time.
class MotionTimeoutError : public SafetyViolationError {
public:
    explicit MotionTimeoutError(
        const std::string& msg,
        std::unordered_map<std::string, std::string> details = {})
        : SafetyViolationError(msg, std::move(details)) {}
};

/// One or more drives reported a motor fault.
class MotorFaultError : public SafetyViolationError {
public:
    explicit MotorFaultError(
        const std::string& msg,
        std::unordered_map<std::string, std::string> details = {})
        : SafetyViolationError(msg, std::move(details)) {}
};

/// A specific arm-level fault condition (e.g. undervoltage, overtemp).
class ArmFault : public MotorFaultError {
public:
    explicit ArmFault(
        const std::string& msg,
        std::unordered_map<std::string, std::string> details = {})
        : MotorFaultError(msg, std::move(details)) {}
};

/// The host-side command watchdog had to take over control.
class WatchdogError : public SafetyViolationError {
public:
    explicit WatchdogError(
        const std::string& msg,
        std::unordered_map<std::string, std::string> details = {})
        : SafetyViolationError(msg, std::move(details)) {}
};

// ── Transport errors ────────────────────────────────────────────────────────

/// CAN/serial transport failed or could not accept a command.
class TransportError : public LiteArmError {
public:
    explicit TransportError(const std::string& msg) : LiteArmError(msg) {}
};

/// CAN bus mode (classic/FD, baud rate) does not match expected configuration.
class CanModeMismatch : public TransportError {
public:
    explicit CanModeMismatch(const std::string& msg) : TransportError(msg) {}
};

/// A motor ID is not registered on the CAN bus.
class CanMotorNotRegistered : public TransportError {
public:
    explicit CanMotorNotRegistered(const std::string& msg) : TransportError(msg) {}
};

/// CAN frame write timed out (bus full or device not responding).
class CanWriteTimeout : public TransportError {
public:
    explicit CanWriteTimeout(const std::string& msg) : TransportError(msg) {}
};

// ── Cancellation ────────────────────────────────────────────────────────────

/// Motion was cancelled by a cancellation token or stop request.
class MotionCancelled : public LiteArmError {
public:
    explicit MotionCancelled(const std::string& msg) : LiteArmError(msg) {}
};

// ── Exception factory ───────────────────────────────────────────────────────

/// Throw the appropriate exception by type name (from wire protocol).
/// Falls back to LiteArmError for unknown types.
/// If the type is a SafetyViolationError subclass, details are attached.
/// This function always throws — it never returns normally.
[[noreturn]] void throw_exception(
    const std::string& type_name,
    const std::string& message,
    std::unordered_map<std::string, std::string> details = {});

/// Check if a type name corresponds to a SafetyViolationError subclass.
bool is_safety_violation_type(const std::string& type_name);

} // namespace litearm
