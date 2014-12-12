package mavlink

import (
	"fmt"
	"unsafe"
)

type Header struct {
	PayloadLength  uint8
	PacketSequence uint8
	SystemID       uint8
	ComponentID    uint8
	MessageID      uint8
}

func (header *Header) BytesRef() []byte {
	return (*[5]byte)(unsafe.Pointer(header))[:]
}

func (header *Header) FieldsShortString() string {
	return fmt.Sprintf("SystemID=%d ComponentID=%d PacketSequence=%d", header.SystemID, header.ComponentID, header.PacketSequence)
}

func (header *Header) FieldsString() string {
	return fmt.Sprintf("SystemID=%d ComponentID=%d PacketSequence=%d PayloadLength=%d", header.SystemID, header.ComponentID, header.PacketSequence, header.PayloadLength)
}

func (header *Header) ShortString() string {
	return fmt.Sprintf("%s{%s}", MessageNameID(header.MessageID), header.FieldsShortString())
}

func (header *Header) String() string {
	return fmt.Sprintf("%s{%s}", MessageNameID(header.MessageID), header.FieldsString())
}
