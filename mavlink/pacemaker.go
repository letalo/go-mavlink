package mavlink

import "time"

type Pacemaker struct {
	conn        *Connection
	componentID uint8
	ticker      *time.Ticker
	stopped     bool
}

func NewPacemaker(conn *Connection, componentID uint8, freq time.Duration, newMsg func() Message) *Pacemaker {
	p := &Pacemaker{
		conn:        conn,
		componentID: componentID,
		ticker:      time.NewTicker(freq),
	}
	go func() {
		for !p.stopped {
			if _, ok := <-p.ticker.C; ok {
				conn.Send(componentID, newMsg(), nil)
			} else {
				time.Sleep(time.Millisecond * 100)
			}
		}
	}()
	return p
}

func (p *Pacemaker) Stop() {
	p.ticker.Stop()
	p.stopped = true
}
