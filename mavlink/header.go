package mavlink

import (
	"fmt"
	"unsafe"

	"github.com/SpaceLeap/go-mavlink/x25"
)

const headerSize = 6

type Header struct {
	FrameStart     uint8
	PayloadLength  uint8
	PacketSequence uint8
	SystemID       uint8
	ComponentID    uint8
	MessageID      uint8
}

func (header *Header) Bytes() []byte {
	return (*[6]byte)(unsafe.Pointer(header))[:]

	// return []byte{
	// 	header.FrameStart,
	// 	header.PayloadLength,
	// 	header.PacketSequence,
	// 	header.SystemID,
	// 	header.ComponentID,
	// 	header.MessageID,
	// }
}

func (header *Header) Hash(hash *x25.Hash) {
	// Skip FrameStart
	hash.WriteByte(header.PayloadLength)
	hash.WriteByte(header.PacketSequence)
	hash.WriteByte(header.SystemID)
	hash.WriteByte(header.ComponentID)
	hash.WriteByte(header.MessageID)
}

func (header *Header) FieldsShortString() string {
	return fmt.Sprintf("System=%d Component=%d Sequence=%d", header.SystemID, header.ComponentID, header.PacketSequence)
}

func (header *Header) FieldsString() string {
	return fmt.Sprintf("System=%d Component=%d Sequence=%d PayloadLength=%d", header.SystemID, header.ComponentID, header.PacketSequence, header.PayloadLength)
}

func (header *Header) ShortString() string {
	return fmt.Sprintf("%s{%s}", MessageNameID(header.MessageID), header.FieldsShortString())
}

func (header *Header) String() string {
	return fmt.Sprintf("%s{%s}", MessageNameID(header.MessageID), header.FieldsString())
}
