package x25

import "io"

var HashStart = Hash{0xffff}

// Warning: Works only on little endian systems
type Hash struct {
	Sum uint16
}

func NewHash() *Hash {
	hash := new(Hash)
	hash.Reset()
	return hash
}

func (hash *Hash) Reset() {
	hash.Sum = 0xffff
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
	var tmp uint8 = data ^ uint8(hash.Sum&0xff)
	tmp ^= (tmp << 4)
	hash.Sum = (hash.Sum >> 8) ^ (uint16(tmp) << 8) ^ (uint16(tmp) << 3) ^ (uint16(tmp) >> 4)
}

type HashedReader struct {
	Hash   *Hash
	reader io.Reader
}

func NewHashedReader(reader io.Reader) *HashedReader {
	return &HashedReader{Hash: NewHash(), reader: reader}
}

func (hr *HashedReader) Read(data []byte) (n int, err error) {
	n, err = hr.reader.Read(data)
	if n > 0 {
		hr.Hash.Write(data[:n])
	}
	return n, err
}

type HashedWriter struct {
	Hash   *Hash
	writer io.Writer
}

func NewHashedWriter(writer io.Writer) *HashedWriter {
	return &HashedWriter{Hash: NewHash(), writer: writer}
}

func (hw *HashedWriter) Write(data []byte) (n int, err error) {
	n, err = hw.writer.Write(data)
	if n > 0 {
		hw.Hash.Write(data[:n])
	}
	return n, err
}
