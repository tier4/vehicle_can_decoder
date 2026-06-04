# Performance Notes

- **Event-driven**: The node processes frames as they arrive from the `can_msgs/Frame` subscription; there is no fixed loop rate to tune.
- **Signal batching**: Signals are accumulated per domain within one CAN frame callback before publishing.
- **Timeout monitoring**: Timeout checks occur at the diagnostics timer rate (`diagnostics_rate_hz`); stale-signal detection resolution is limited by that period.
- **Transform compilation**: Expressions are compiled once at startup; evaluation is fast.
