package mavlink

type Message interface {
	MsgID() uint8    // ID of the message. Using Msg prefix to avoid name collisions.
	MsgName() string // Name of the message. Using Msg prefix to avoid name collisions.
	MsgSize() uint8  // Size in bytes of the message. Using Msg prefix to avoid name collisions.
}

type NewMessageFunc func(messageID uint8) Message
