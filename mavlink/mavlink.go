package mavlink

import (
	"encoding/binary"
	"io"
	"log"

	"github.com/SpaceLeap/go-mavlink/x25"
)

const (
	FRAME_START = 0xFE
)

// The following variables will be set by importing
// a protocol definition package.
var (
	ProtocolName    string
	ProtocolVersion string
	MessageFactory  [256]func() Message
)

func Send(writer io.Writer, systemID, componentID, sequence uint8, message Message) error {
	_, err := NewPacket(systemID, componentID, sequence, message).WriteTo(writer)
	return err
}

func Receive(reader io.Reader) (*Packet, error) {
	var packet Packet

	// Slice bytes are pointer to packet.Header,
	// so accessing the slice elements accesses the packet.Header
	headerBytes := packet.Header.Bytes()
	firstHeaderByte := headerBytes[:1] // points to packet.Header.FrameStart

	// Initially FrameStart will be zero and != FRAME_START
	// Read until we get FRAME_START, silently ignore non FRAME_START bytes
	for packet.Header.FrameStart != FRAME_START {
		_, err := reader.Read(firstHeaderByte)
		if err != nil {
			return nil, err
		}
	}

	hash := x25.HashStart
	hashedReader := x25.HashedStream{Hash: &hash, Reader: reader}

	// Read rest of header directly into packet.Header referenced by headerBytes
	_, err := io.ReadFull(&hashedReader, headerBytes[1:])
	if err != nil {
		return nil, err
	}

	// to do: check component sequence

	msg := MessageFactory[packet.Header.MessageID]()
	if msg == nil {
		return nil, ErrUnknownMessageID(packet.Header.MessageID)
	}
	if packet.Header.PayloadLength != msg.TypeSize() {
		return nil, ErrInvalidPayloadLength(packet.Header.PayloadLength)
	}

	err = binary.Read(&hashedReader, binary.LittleEndian, msg)
	if msg == nil {
		return nil, err
	}

	err = binary.Read(reader, binary.LittleEndian, &packet.Checksum)
	if msg == nil {
		return nil, err
	}

	if packet.Checksum != hash.Sum() {
		return nil, ErrInvalidChecksum(hash.Sum())
	}

	return nil, nil
}

// ReceiveNoErr calls Receive and returns a *Packet if there was no error.
// If Receive returns an error, and logger is not nil,
// then the error will be logged with logger.
// The only exception is the error io.EOF. In that case the error
// won't be logged and a nil packet will be returned.
func ReceiveNoErr(reader io.Reader, logger *log.Logger) (packet *Packet) {
	var err error
	for packet == nil {
		packet, err = Receive(reader)
		if err != nil {
			if err == io.EOF {
				return nil
			}
			if logger != nil {
				logger.Println(err)
			}
		}
	}
	return packet
}
