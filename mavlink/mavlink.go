package mavlink

import (
	// "bytes"
	"encoding/binary"
	"errors"
	"fmt"
	"io"
	"log"

	"github.com/SpaceLeap/go-mavlink/x25"
)

const FRAME_START = 0xFE

var FRAME_START_BYTE = []byte{FRAME_START}

// The following variables will be set by importing
// a protocol definition package.
var (
	ProtocolName    string
	ProtocolVersion uint8
	MessageFactory  [256]func() Message
)

var (
	SendLogger             *log.Logger
	ReceiveLogger          *log.Logger
	AllErrorsLogger        *log.Logger
	UnreportedErrorsLogger *log.Logger
	DebugLogger            *log.Logger
)

func Send(writer io.Writer, systemID, componentID, sequence uint8, message Message) error {
	_, err := NewPacket(systemID, componentID, sequence, message).WriteTo(writer)
	return err
}

func Receive(reader io.Reader) (*Packet, error) {

	firstByte := make([]byte, 1)

	_, err := reader.Read(firstByte)
	if err != nil {
		return nil, err
	}

	// Read until we get FRAME_START
	skipped := 0
	for firstByte[0] != FRAME_START {
		_, err := reader.Read(firstByte)
		if err != nil {
			return nil, err
		}
		skipped++
		if skipped > 1024 {
			return nil, errors.New("Receive wasn't able to find FRAME_START within 1kB, giving up now")
		}
	}

	if skipped != 0 && UnreportedErrorsLogger != nil {
		UnreportedErrorsLogger.Printf("Receive skipped %d bytes to find FRAME_START", skipped)
	}

	var (
		packet Packet

		// Slice uses packet.Header for data storage,
		// so accessing the slice elements accesses the packet.Header
		headerBytesRef = packet.Header.BytesRef()

		hashedReader = x25.MakeHashedReader(reader)
	)

	// Read rest of header directly into packet.Header referenced by headerBytesRef
	n, err := io.ReadFull(&hashedReader, headerBytesRef)
	if err != nil {
		return nil, fmt.Errorf("Read %d of %d header bytes. %s", n, len(headerBytesRef), err)
	}

	// log.Println("HEADER", headerBytesRef)

	// to do: check component sequence

	packet.Message = MessageFactory[packet.Header.MessageID]()
	if packet.Message == nil {
		io.ReadFull(reader, make([]byte, packet.Message.TypeSize())) // Skip rest of message
		return nil, ErrUnknownMessageID(packet.Header.MessageID)
	}
	if packet.Header.PayloadLength != packet.Message.TypeSize() {
		skip := packet.Header.PayloadLength
		if skip > packet.Message.TypeSize() {
			skip = packet.Message.TypeSize() // use smaller size
		}
		io.ReadFull(reader, make([]byte, skip)) // Skip rest of message
		err = fmt.Errorf(
			"Expected %d as PayloadLength for message type %s, got %d",
			packet.Message.TypeSize(),
			MessageName(packet.Header.MessageID),
			packet.Header.PayloadLength)
		return nil, err
	}
	// Just for debugging:
	if false {
		if binary.Size(packet.Message) != int(packet.Message.TypeSize()) {
			panic("Invalid packet size")
		}
	}

	err = binary.Read(&hashedReader, binary.LittleEndian, packet.Message)
	if err != nil {
		return nil, err
	}

	hashedReader.Hash.WriteByte(packet.Message.TypeCRCExtra())

	var (
		calculatedChecksum = hashedReader.Hash.Sum
		receivedChecksum   uint16
	)

	err = binary.Read(reader, binary.LittleEndian, &receivedChecksum)
	if err != nil {
		return nil, err
	}

	if receivedChecksum != calculatedChecksum {
		err = &ErrInvalidChecksum{
			Packet:             &packet,
			CalculatedChecksum: calculatedChecksum,
			ReceivedChecksum:   receivedChecksum,
		}
		return nil, err
	}

	return &packet, nil
}

// ReceiveLogErr calls Receive and returns a *Packet if there was no error.
// If Receive returns an error, and LogUnreportedErrors is true,
// then the error will be logged.
// The only exception is the error io.EOF. In that case the error
// won't be logged and a nil packet will be returned.
func ReceiveLogErr(reader io.Reader) (packet *Packet) {
	var err error
	for packet == nil {
		packet, err = Receive(reader)
		if err != nil {
			if err == io.EOF {
				return nil
			}
			if UnreportedErrorsLogger != nil && AllErrorsLogger == nil {
				UnreportedErrorsLogger.Println(err)
			}
		}
	}
	return packet
}
