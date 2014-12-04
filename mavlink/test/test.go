package test

import (
	"github.com/SpaceLeap/go-mavlink/mavlink"
)

const (
	PROTOCOL_NAME    = "test"
	PROTOCOL_VERSION = 3
)

func init() {
	mavlink.ProtocolName = PROTOCOL_NAME
	mavlink.ProtocolVersion = PROTOCOL_VERSION

	mavlink.MessageFactory[0] = func() mavlink.Message { return new(TestTypes) }
}

////////////////////////////////////////////////////////////////////////////////
// Enums
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Messages
////////////////////////////////////////////////////////////////////////////////

// Test all field types
type TestTypes struct {
	DArray   [3]float64 // double_array
	S64Array [3]int64   // int64_t_array
	U64Array [3]uint64  // uint64_t_array
	D        float64    // double
	S64      int64      // int64_t
	U64      uint64     // uint64_t
	FArray   [3]float32 // float_array
	S32Array [3]int32   // int32_t_array
	U32Array [3]uint32  // uint32_t_array
	F        float32    // float
	S32      int32      // int32_t
	U32      uint32     // uint32_t
	S16Array [3]int16   // int16_t_array
	U16Array [3]uint16  // uint16_t_array
	S16      int16      // int16_t
	U16      uint16     // uint16_t
	S8Array  [3]int8    // int8_t_array
	U8Array  [3]uint8   // uint8_t_array
	S8       int8       // int8_t
	U8       uint8      // uint8_t
	S        Char10     // string
	C        byte       // char
}

func (self *TestTypes) TypeID() uint8 {
	return 0
}

func (self *TestTypes) TypeName() string {
	return "TEST_TYPES"
}

func (self *TestTypes) TypeSize() uint8 {
	return 86
}

func (self *TestTypes) TypeCRCExtra() uint8 {
	return 113
}

////////////////////////////////////////////////////////////////////////////////
// String Helpers
////////////////////////////////////////////////////////////////////////////////

func truncate(chars []byte) []byte {
	for i, c := range chars {
		if c == 0 {
			return chars[:i]
		}
	}
	return chars
}

type Char3 [3]byte

func (chars *Char3) String() string {
	return string(truncate(chars[:]))
}

type Char10 [10]byte

func (chars *Char10) String() string {
	return string(truncate(chars[:]))
}
