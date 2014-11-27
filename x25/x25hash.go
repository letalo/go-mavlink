package x25

import "io"

var HashStart = Hash{0xffff}

// Warning: Works only on little endian systems
type Hash struct {
	sum uint16
}

func NewHash() *Hash {
	hash := new(Hash)
	hash.Reset()
	return hash
}

func (hash *Hash) Reset() {
	hash.sum = 0xffff
}

func (hash *Hash) Write(data []byte) (n int, err error) {
	n = len(data)
	for i := 0; i < n; i++ {
		hash.WriteByte(data[i])
	}
	return n, nil
}

// Warning: Works only on little endian systems
func (hash *Hash) WriteByte(data byte) {
	tmp := data ^ uint8(hash.sum&0xff)
	tmp ^= (tmp << 4)
	hash.sum = (hash.sum >> 8) ^ (uint16(tmp) << 8) ^ (uint16(tmp) << 3) ^ (uint16(tmp) >> 4)
}

func (hash *Hash) Sum() uint16 {
	return hash.sum
}

// HashedStream is used to hash the data that is
// read or written via Reader and Writer.
type HashedStream struct {
	Hash   *Hash
	Reader io.Reader
	Writer io.Writer
}

func (stream *HashedStream) Read(data []byte) (n int, err error) {
	n, err = stream.Read(data)
	if n > 0 {
		stream.Hash.Write(data[:n])
	}
	return n, err
}

func (stream *HashedStream) Write(data []byte) (n int, err error) {
	stream.Hash.Write(data)
	return stream.Writer.Write(data)
}
