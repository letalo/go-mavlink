package x25

import (
	// "fmt"
	"testing"
)

func TestHash(t *testing.T) {

	goHash := NewHash()
	cHash := goHash.Sum

	for i := uint8(0); i < 255; i++ {
		goHash.WriteByte(i)
		crc_accumulate(i, &cHash)
		if goHash.Sum != cHash {
			t.Fail()
		}
	}

}
