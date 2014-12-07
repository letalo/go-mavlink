package mavlink

import "fmt"

type ErrUnknownMessageID uint8

func (id ErrUnknownMessageID) Error() string {
	return fmt.Sprintf("Unknow message ID %d", id)
}

type ErrPacketLoss int

func (loss ErrPacketLoss) Error() string {
	return fmt.Sprintf("Lost %d packages", loss)
}

type ErrInvalidChecksum struct {
	Packet             *Packet
	CalculatedChecksum uint16
	ReceivedChecksum   uint16
}

func (e *ErrInvalidChecksum) Error() string {
	return fmt.Sprintf("Calculated checksum %d does not match received checksum %d. Packet %s", e.CalculatedChecksum, e.ReceivedChecksum, e.Packet)
}
