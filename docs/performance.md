# Performance Notes

- **Event-driven**: The node processes frames as they arrive from the `can_msgs/Frame` subscription; there is no fixed loop rate to tune.
- **Signal batching**: Signals are accumulated per domain within one timer tick before publishing.
- **Timeout monitoring**: Timeout checks occur at the loop rate; resolution is limited by loop period.
- **Transform compilation**: Expressions are compiled once at startup; evaluation is fast.
