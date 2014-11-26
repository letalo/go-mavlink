package mavlink

import (
	"bytes"
	"encoding/binary"
	"io"

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
}

func NewPacket(systemID, componentID uint8, message Message) (packet *Packet) {
	componentPacketSequences[componentID]++
	return &Packet{
		Header: Header{
			FrameStart:     FRAME_START,
			PayloadLength:  message.Size(),
			PacketSequence: componentPacketSequences[componentID],
			SystemID:       systemID,
			ComponentID:    componentID,
			MessageID:      message.ID(),
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
	n += int64(packet.Message.Size())

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
	hash.WriteByte(MessageCRSs[packet.Header.MessageID])

	return hash.Sum()
}

func (packet *Packet) Bytes() []byte {
	buf := bytes.NewBuffer(packet.Header.Bytes())
	binary.Write(buf, binary.LittleEndian, packet.Message)
	binary.Write(buf, binary.LittleEndian, packet.Checksum)
	return buf.Bytes()
}
