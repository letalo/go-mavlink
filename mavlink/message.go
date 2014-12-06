package mavlink

import "fmt"

type Message interface {
	TypeID() uint8       // ID of the message
	TypeName() string    // Name of the message
	TypeSize() uint8     // Size in bytes of the message
	TypeCRCExtra() uint8 // CRC_EXTRA
	FieldsString() string
	String() string
}

func MessageName(id uint8) string {
	if f := MessageFactory[id]; f != nil {
		return f().TypeName()
	}
	return ""
}

func MessageNameID(id uint8) string {
	var msg Message
	if f := MessageFactory[id]; f != nil {
		msg = f()
	}
	if msg == nil {
		return fmt.Sprintf("UNKNOWN(%d)", id)
	}
	return NameIDFromMessage(msg)
}

func NameIDFromMessage(msg Message) string {
	if msg == nil {
		return "UNKNOWN(?)"
	}
	return fmt.Sprintf("%s(%d)", msg.TypeName(), msg.TypeID())
}
