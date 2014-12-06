package mavlink

import (
	"io"
	"net"
	"strconv"
	"strings"

	"github.com/ungerik/go-dry"
	"github.com/ungerik/goserial"
)

// Dial opens a connection of a type depending of the schema of the address.
// The supported connection types are serial, UDP, and Unix sockets.
func Dial(address string) (io.ReadWriteCloser, error) {
	name, ext := dry.StringSplitOnceRune(address, ':')
	if serial.IsName(name) {
		baud := serial.Baud115200
		if ext != "" {
			i, err := strconv.Atoi(ext)
			if err != nil {
				return nil, err
			}
			baud = serial.Baud(i)
		}
		return serial.OpenDefault(name, baud, 0)
	}

	if strings.HasPrefix(address, "/") {
		return net.Dial("unix", address)
	}

	return net.Dial("udp", address)
}
