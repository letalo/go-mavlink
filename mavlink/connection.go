package mavlink

import (
	"io"
)

type Connection struct {
	io.ReadWriter
	localComponentSeq  [256]uint8
	remoteComponentSeq [256]uint8
	systemID           uint8
}

func NewConnection(wrappedConn io.ReadWriter, systemID uint8) *Connection {
	return &Connection{ReadWriter: wrappedConn, systemID: systemID}
}

func (conn *Connection) Send(componentID uint8, message Message) error {
	conn.localComponentSeq[componentID]++
	return Send(conn, conn.systemID, componentID, conn.localComponentSeq[componentID], message)
}

func (conn *Connection) Receive() (packet *Packet, loss int, err error) {
	packet, err = Receive(conn)
	if err != nil {
		return nil, 0, err
	}
	component := packet.Header.ComponentID
	loss = int(packet.Header.PacketSequence) - int(conn.remoteComponentSeq[component]) - 1
	if loss < 0 {
		loss = 255 - loss
	}
	return packet, loss, nil
}

func (conn *Connection) Close() error {
	if closer, ok := conn.ReadWriter.(io.Closer); ok {
		return closer.Close()
	}
	return nil
}
