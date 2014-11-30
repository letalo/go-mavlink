package mavlink

import (
	"io"
	"log"
	"time"
)

type Connection struct {
	wrappedConn        io.ReadWriter
	channel            chan *Packet
	closeTime          chan time.Time
	closeError         chan error
	closed             bool
	localComponentSeq  [256]uint8
	remoteComponentSeq [256]uint8
	systemID           uint8
}

func NewConnection(wrappedConn io.ReadWriter, systemID uint8) *Connection {
	conn := &Connection{
		wrappedConn: wrappedConn,
		channel:     make(chan *Packet, 64),
		closeTime:   make(chan time.Time, 1),
		closeError:  make(chan error),
		systemID:    systemID,
	}
	go conn.sendLoop()
	go conn.receiveLoop()
	return conn
}

func (conn *Connection) receiveLoop() {
	for !conn.closed {

		packet, err := Receive(conn.wrappedConn)
		if err != nil {
			if LogAllErrors {
				log.Println(err)
			}
			if err == io.EOF {
				conn.close() // make sure everything is in closed state
				return
			} else {
				conn.channel <- &Packet{Err: err}
				continue
			}
		}

		component := packet.Header.ComponentID
		loss := int(packet.Header.PacketSequence) - int(conn.remoteComponentSeq[component]) - 1
		if loss < 0 {
			loss = 255 - loss
		}
		if loss > 0 {
			err := ErrPacketLoss(loss)
			if LogAllErrors {
				log.Println(err)
			}
			conn.channel <- &Packet{Err: err}
		}

		conn.channel <- packet
	}
}

func (conn *Connection) sendLoop() {
	for !conn.closed {
		select {
		case packet := <-conn.channel:
			conn.localComponentSeq[packet.Header.ComponentID]++
			packet.Header.PacketSequence = conn.localComponentSeq[packet.Header.ComponentID]

			_, packet.Err = packet.WriteTo(conn.wrappedConn)
			if packet.Err != nil {
				if LogAllErrors {
					log.Println(packet.Err)
				}
				if packet.OnErr != nil {
					packet.OnErr(packet)
				}
			}
		default:
			time.Sleep(time.Millisecond)
		}
	}
}

func (conn *Connection) Send(componentID uint8, message Message, onErr func(*Packet)) {
	if conn.closed {
		return
	}

	packet := NewPacket(conn.systemID, componentID, 0, message)
	packet.OnErr = onErr

	conn.channel <- packet
}

func (conn *Connection) Receive() *Packet {
	if conn.closed {
		return nil
	}

	return <-conn.channel
}

func (conn *Connection) close() (err error) {
	defer func() { conn.closed = true }()

	if closer, ok := conn.wrappedConn.(io.Closer); ok {
		err = closer.Close()
		if err != nil && LogAllErrors {
			log.Println(err)
		}
	}

	return err
}

func (conn *Connection) Close(timeout time.Time) error {
	return conn.close()
	// conn.closeTime <- timeout
	// return <-conn.closeError
}
