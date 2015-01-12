package mavlink

import (
	"bytes"
	"encoding/binary"
	"errors"
	"fmt"
	"io"
	// "log"
	"strconv"
	"strings"

	"github.com/SpaceLeap/go-mavlink/x25"
	"github.com/ungerik/go-dry"
)

var errConnectionClosed = errors.New("Connection closed")

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

func ParsePacket(s string) (packet *Packet, err error) {
	name, rest := dry.StringSplitOnceChar(s, '(')
	id, rest := dry.StringSplitOnceChar(rest, ')')
	i, err := strconv.Atoi(id)
	if err != nil || i > 255 {
		return nil, errors.New("Invalid message ID " + id)
	}

	packet = new(Packet)
	packet.Message = MessageFactory[i]()
	if packet.Message == nil {
		return nil, fmt.Errorf("Unsupported message type: %s(%s)", name, id)
	}
	if packet.Message.TypeName() != name {
		return nil, errors.New("Message ID and name do not match")
	}
	packet.Header.MessageID = uint8(i)
	packet.Header.PayloadLength = packet.Message.TypeSize()

	if len(rest) < 2 || rest[0] != '{' || rest[len(rest)-1] != '}' {
		return nil, errors.New("Missing {} after message type")
	}
	s = rest[1 : len(rest)-1]

	for _, arg := range strings.Split(s, " ") {
		name, value := dry.StringSplitOnceChar(arg, '=')
		value = strings.Trim(value, `"'`)
		switch name {
		case "PayloadLength", "MessageID":
			continue

		case "PacketSequence", "SystemID", "ComponentID":
			err = dry.ReflectSetStructFieldString(&packet.Header, name, value)
			if err != nil {
				return nil, err
			}

		default:
			err = dry.ReflectSetStructFieldString(packet.Message, name, value)
			if err != nil {
				return nil, err
			}
		}
	}

	return packet, nil
}

func (packet *Packet) ReadFrom(reader io.Reader) (n int64, err error) {
	if packet == nil {
		panic("ReadFrom called at nil Packet")
	}

	var firstByteStorage [1]byte
	firstByte := firstByteStorage[:]

	m, err := reader.Read(firstByte)
	n = int64(m)
	if err != nil {
		return n, err
	} else if m == 0 {
		return n, errConnectionClosed
	}

	// Read until we get FRAME_START
	var skipped int
	for firstByte[0] != FRAME_START {
		m, err = reader.Read(firstByte)
		n += int64(m)
		if err != nil {
			return n, err
		} else if m == 0 {
			return n, errConnectionClosed
		}
		skipped++
		if skipped > 1024 {
			return n, errors.New("Receive wasn't able to find FRAME_START within 1kB, giving up now")
		}
	}

	if skipped > 0 && UnreportedErrorsLogger != nil {
		UnreportedErrorsLogger.Printf("Receive skipped %d bytes to find FRAME_START", skipped)
	}

	var (
		// Slice uses packet.Header for data storage,
		// so accessing the slice elements accesses the packet.Header
		// headerBytesRef = packet.Header.BytesRef()
		headerBytes = make([]byte, 5)

		hashedReader = x25.MakeHashedReader(reader)
	)

	// Read rest of header directly into packet.Header referenced by headerBytesRef
	m, err = io.ReadFull(&hashedReader, headerBytes)
	n += int64(m)
	if err != nil {
		return n, fmt.Errorf("Read %d of 5 remaining header bytes. %s", n, err)
	} else if m == 0 {
		return n, errConnectionClosed
	}

	packet.Header.SetBytes(headerBytes)

	// log.Println("HEADER", append(firstByte, headerBytes...))

	// to do: check component sequence

	f := MessageFactory[packet.Header.MessageID]
	if f == nil {
		m, _ = io.ReadFull(reader, make([]byte, packet.Header.PayloadLength+2)) // Skip rest of message
		n += int64(m)
		return n, fmt.Errorf("Unknown message ID %d", packet.Header.MessageID)
	}
	packet.Message = f()
	if packet.Message == nil {
		m, _ = io.ReadFull(reader, make([]byte, packet.Message.TypeSize())) // Skip rest of message
		n += int64(m)
		return n, ErrUnknownMessageID(packet.Header.MessageID)
	}
	if packet.Header.PayloadLength != packet.Message.TypeSize() {
		// Header does not match of what we know about the message,
		// so let's skip this message
		skip := packet.Header.PayloadLength
		if skip > packet.Message.TypeSize() {
			// use smaller size to minimize data loss when we don't know which number is true
			skip = packet.Message.TypeSize()
		}
		m, _ = io.ReadFull(reader, make([]byte, skip+2)) // Skip rest of message
		n += int64(m)
		err = fmt.Errorf(
			"Expected %d as PayloadLength for message type %s(%d), got %d",
			packet.Message.TypeSize(),
			MessageName(packet.Header.MessageID),
			packet.Header.MessageID,
			packet.Header.PayloadLength)
		return n, err
	}
	// Just for debugging:
	if false {
		if binary.Size(packet.Message) != int(packet.Message.TypeSize()) {
			panic("Invalid packet size")
		}
	}

	m, err = dry.ReadBinary(&hashedReader, binary.LittleEndian, packet.Message)
	n += int64(m)
	if err != nil {
		return n, err
	} else if m == 0 {
		return n, errConnectionClosed
	}

	hashedReader.Hash.WriteByte(packet.Message.TypeCRCExtra())

	var (
		calculatedChecksum = hashedReader.Hash.Sum
		receivedChecksum   uint16
	)

	m, err = dry.ReadBinary(reader, binary.LittleEndian, &receivedChecksum)
	n += int64(m)
	if err != nil {
		return n, err
	} else if m == 0 {
		return n, errConnectionClosed
	}

	if receivedChecksum != calculatedChecksum {
		err = &ErrInvalidChecksum{
			Packet:             packet,
			CalculatedChecksum: calculatedChecksum,
			ReceivedChecksum:   receivedChecksum,
		}
		return n, err
	}

	return n, nil
}

func (packet *Packet) WriteTo(writer io.Writer) (n int64, err error) {
	m, err := writer.Write(FRAME_START_BYTE)
	n = int64(m)
	if err != nil {
		return n, err
	}

	hashedWriter := x25.MakeHashedWriter(writer)

	// Write and hash header
	m, err = hashedWriter.Write(packet.Header.Bytes())
	n += int64(m)
	if err != nil {
		return n, err
	}

	// Write and hash message
	err = binary.Write(&hashedWriter, binary.LittleEndian, packet.Message)
	if err != nil {
		return n, err
	}
	n += int64(packet.Message.TypeSize())

	// Add CRCExtra to the hash
	hashedWriter.Hash.WriteByte(packet.Message.TypeCRCExtra())

	hashBytes := make([]byte, 2)
	binary.LittleEndian.PutUint16(hashBytes, hashedWriter.Hash.Sum)

	// Write hash
	m, err = writer.Write(hashBytes)
	n += int64(m)

	return n, err
}

// func (packet *Packet) WriteTo(writer io.Writer) (n int64, err error) {
// 	m, err := writer.Write(packet.WireBytes())
// 	return int64(m), err
// }

func (packet *Packet) WireSize() int {
	return 6 + int(packet.Message.TypeSize()) + 2
}

func (packet *Packet) WireBytes() []byte {
	buf := bytes.NewBuffer(make([]byte, 0, packet.WireSize()))
	buf.WriteByte(FRAME_START)

	writer := x25.MakeHashedWriter(buf)
	writer.Write(packet.Header.Bytes())
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
