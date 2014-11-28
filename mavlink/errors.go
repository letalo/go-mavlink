package mavlink

import "fmt"

type ErrUnknownMessageID uint8

func (id ErrUnknownMessageID) Error() string {
	return fmt.Sprintf("Unknow message ID %d", id)
}

type ErrInvalidChecksum uint16

func (sum ErrInvalidChecksum) Error() string {
	return fmt.Sprintf("Invalid checksum %d", sum)
}

type ErrInvalidPayloadLength uint8

func (length ErrInvalidPayloadLength) Error() string {
	return fmt.Sprintf("Message ID and PayloadLength (%d) don't match", length)
}

type ErrPacketLoss int

func (loss ErrPacketLoss) Error() string {
	return fmt.Sprintf("Lost %d packages", loss)
}

// type ErrInvalidStartframe uint8

// func (frame ErrInvalidStartframe) Error() string {
// 	return fmt.Sprintf("Invalid start of frame %d", frame)
// }
