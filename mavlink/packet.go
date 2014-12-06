package mavlink

import (
	"bytes"
	"encoding/binary"
	// "fmt"
	"io"
	// "strconv"
	// "strings"

	"github.com/SpaceLeap/go-mavlink/x25"
)

const (
	x25InitCRC uint16 = 0xffff
	// x25ValidateCRC uint16 = 0xf0b8
)

type Packet struct {
	Header   Header
	Message  Message
	Checksum uint16
	Err      error
	OnErr    func(*Packet)
}

func NewPacket(systemID, componentID, sequence uint8, message Message) (packet *Packet) {
	return &Packet{
		Header: Header{
			FrameStart:     FRAME_START,
			PayloadLength:  message.TypeSize(),
			PacketSequence: sequence,
			SystemID:       systemID,
			ComponentID:    componentID,
			MessageID:      message.TypeID(),
		},
		Message: message,
	}
}

func (packet *Packet) WriteTo(writer io.Writer) (n int64, err error) {
	hash := x25.HashStart
	hashedWriter := x25.HashedStream{Hash: &hash, Writer: writer}

	m, err := writer.Write(packet.Header.Bytes())
	n = int64(m)
	if err != nil {
		return n, err
	}
	packet.Header.Hash(&hash)

	err = binary.Write(&hashedWriter, binary.LittleEndian, packet.Message)
	if err != nil {
		return n, err
	}
	n += int64(packet.Message.TypeSize())

	packet.Checksum = hash.Sum()

	var hashBytesArray [2]byte // local variable to avoid allocation on heap
	hashBytes := hashBytesArray[:]
	binary.LittleEndian.PutUint16(hashBytes, packet.Checksum)

	m, err = writer.Write(hashBytes)
	n += int64(m)

	return n, err
}

func (packet *Packet) ComputeChecksum() uint16 {
	hash := x25.HashStart

	packet.Header.Hash(&hash)
	binary.Write(&hash, binary.LittleEndian, packet.Message)
	hash.WriteByte(packet.Message.TypeCRCExtra())

	return hash.Sum()
}

func (packet *Packet) Bytes() []byte {
	buf := bytes.NewBuffer(packet.Header.Bytes())
	binary.Write(buf, binary.LittleEndian, packet.Message)
	binary.Write(buf, binary.LittleEndian, packet.Checksum)
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
