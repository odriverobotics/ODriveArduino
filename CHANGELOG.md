# [0.10.6]
- Fixed getEndpoint not updating requested_msg_id_, causing periodic messages to be read as a TxSdo message.

# [0.10.5]
- Fixed SineWaveCAN example vbus request timeout

# [0.10.4]

- Updated can_helpers to remove some UB and make code run on Arduino boards more reliably

# [0.10.3]

- Fixed CAN awaitMsg timeout; now actually milliseconds instead of seconds
