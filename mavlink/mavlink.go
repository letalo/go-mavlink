package mavlink

import (
	// "bytes"
	"io"
	"log"
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

func LogEverything(logger *log.Logger) {
	SendLogger = logger
	ReceiveLogger = logger
	AllErrorsLogger = logger
	UnreportedErrorsLogger = logger
	DebugLogger = logger
}

func Send(writer io.Writer, systemID, componentID, sequence uint8, message Message) error {
	_, err := NewPacket(systemID, componentID, sequence, message).WriteTo(writer)
	return err
}

func Receive(reader io.Reader) (*Packet, error) {
	var packet Packet
	_, err := packet.ReadFrom(reader)
	if err != nil {
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
