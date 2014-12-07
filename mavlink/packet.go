package mavlink

import (
	"bytes"
	"encoding/binary"
	"io"

	"github.com/SpaceLeap/go-mavlink/x25"
	"github.com/ungerik/go-dry"
)

type Packet struct {
	Header  Header
	Message Message
	Err     error
	OnErr   func(*Packet)
}

func NewPacket(systemID, componentID, sequence uint8, message Message) (packet *Packet) {
	return &Packet{
		Header: Header{
			PayloadLength:  message.TypeSize(),
			PacketSequence: sequence,
			SystemID:       systemID,
			ComponentID:    componentID,
			MessageID:      message.TypeID(),
		},
		Message: message,
	}
}

// func (packet *Packet) WriteTo(writer io.Writer) (n int64, err error) {
// 	m, err := writer.Write(FRAME_START_BYTE)
// 	n = int64(m)
// 	if err != nil {
// 		return n, err
// 	}

// 	hashedWriter := x25.MakeHashedWriter(writer)

// 	// Write and hash header
// 	m, err = hashedWriter.Write(packet.Header.BytesRef())
// 	n += int64(m)
// 	if err != nil {
// 		return n, err
// 	}

// 	// Write and hash message
// 	err = binary.Write(&hashedWriter, binary.LittleEndian, packet.Message)
// 	if err != nil {
// 		return n, err
// 	}
// 	n += int64(packet.Message.TypeSize())

// 	// Add CRCExtra to the hash
// 	hashedWriter.Hash.WriteByte(packet.Message.TypeCRCExtra())

// 	hashBytes := make([]byte, 2)
// 	binary.LittleEndian.PutUint16(hashBytes, hashedWriter.Hash.Sum)

// 	// Write hash
// 	m, err = writer.Write(hashBytes)
// 	n += int64(m)

// 	return n, err
// }

func (packet *Packet) WriteTo(writer io.Writer) (n int64, err error) {
	m, err := writer.Write(packet.WireBytes())
	return int64(m), err
}

func (packet *Packet) WireSize() int {
	return 6 + int(packet.Message.TypeSize()) + 2
}

func (packet *Packet) WireBytes() []byte {
	buf := bytes.NewBuffer(make([]byte, 0, packet.WireSize()))
	buf.WriteByte(FRAME_START)

	writer := x25.MakeHashedWriter(buf)
	writer.Write(packet.Header.BytesRef())
	binary.Write(&writer, binary.LittleEndian, packet.Message)
	writer.Hash.WriteByte(packet.Message.TypeCRCExtra())

	least, most := dry.EndianSafeSplitUint16(writer.Hash.Sum)
	buf.WriteByte(least)
	buf.WriteByte(most)

	return buf.Bytes()
}

func (packet *Packet) String() string {
	var buf bytes.Buffer
	buf.WriteString(MessageNameID(packet.Header.MessageID))
	buf.WriteByte('{')
	buf.WriteString(packet.Header.FieldsString())
	if packet.Message != nil {
		buf.WriteByte(' ')
		buf.WriteString(packet.Message.FieldsString())
	} else {
		buf.WriteString(" Message=nil")
	}
	buf.WriteByte('}')
	return buf.String()
}

func (packet *Packet) ShortString() string {
	var buf bytes.Buffer
	buf.WriteString(MessageNameID(packet.Header.MessageID))
	buf.WriteByte('{')
	buf.WriteString(packet.Header.FieldsShortString())
	if packet.Message != nil {
		buf.WriteByte(' ')
		buf.WriteString(packet.Message.FieldsString())
	} else {
		buf.WriteString(" Message=nil")
	}
	buf.WriteByte('}')
	return buf.String()
}
