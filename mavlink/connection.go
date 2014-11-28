package mavlink

import (
	"io"
)

type Connection struct {
	io.ReadWriter
	localComponentSeq  [256]uint8
	remoteComponentSeq [256]uint8
	bufferedPacket     *Packet
	systemID           uint8
}

func NewConnection(wrappedConn io.ReadWriter, systemID uint8) *Connection {
	return &Connection{ReadWriter: wrappedConn, systemID: systemID}
}

func (conn *Connection) Send(componentID uint8, message Message) error {
	conn.localComponentSeq[componentID]++
	return Send(conn, conn.systemID, componentID, conn.localComponentSeq[componentID], message)
}

func (conn *Connection) Receive() (packet *Packet, err error) {
	if conn.bufferedPacket != nil {
		packet, conn.bufferedPacket = conn.bufferedPacket, nil
		return packet, nil
	}
	packet, err = Receive(conn)
	if err != nil {
		return nil, err
	}
	component := packet.Header.ComponentID
	loss := int(packet.Header.PacketSequence) - int(conn.remoteComponentSeq[component]) - 1
	if loss < 0 {
		loss = 255 - loss
	}
	if loss > 0 {
		conn.bufferedPacket = packet
		return nil, ErrPacketLoss(loss)
	}
	return packet, nil
}

func (conn *Connection) Close() error {
	if closer, ok := conn.ReadWriter.(io.Closer); ok {
		return closer.Close()
	}
	return nil
}
