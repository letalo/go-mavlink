package test

import (
	"fmt"

	"github.com/SpaceLeap/go-mavlink/mavlink"
)

const (
	PROTOCOL_NAME    = "test"
	PROTOCOL_VERSION = 3
)

func Init() {
	for i := range mavlink.MessageFactory {
		mavlink.MessageFactory[i] = nil
	}

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
	U64      uint64     // uint64_t
	S64      int64      // int64_t
	D        float64    // double
	U64Array [3]uint64  // uint64_t_array
	S64Array [3]int64   // int64_t_array
	DArray   [3]float64 // double_array
	U32      uint32     // uint32_t
	S32      int32      // int32_t
	F        float32    // float
	U32Array [3]uint32  // uint32_t_array
	S32Array [3]int32   // int32_t_array
	FArray   [3]float32 // float_array
	U16      uint16     // uint16_t
	S16      int16      // int16_t
	U16Array [3]uint16  // uint16_t_array
	S16Array [3]int16   // int16_t_array
	C        byte       // char
	S        Char10     // string
	U8       uint8      // uint8_t
	S8       int8       // int8_t
	U8Array  [3]uint8   // uint8_t_array
	S8Array  [3]int8    // int8_t_array
}

func (self *TestTypes) TypeID() uint8 {
	return 0
}

func (self *TestTypes) TypeName() string {
	return "TEST_TYPES"
}

func (self *TestTypes) TypeSize() uint8 {
	return 179
}

func (self *TestTypes) TypeCRCExtra() uint8 {
	return 201
}

func (self *TestTypes) FieldsString() string {
	return fmt.Sprintf("U64=%d S64=%d D=%f U64Array=%v S64Array=%v DArray=%v U32=%d S32=%d F=%f U32Array=%v S32Array=%v FArray=%v U16=%d S16=%d U16Array=%v S16Array=%v C=%d S=\"%s\" U8=%d S8=%d U8Array=%v S8Array=%v", self.U64, self.S64, self.D, self.U64Array, self.S64Array, self.DArray, self.U32, self.S32, self.F, self.U32Array, self.S32Array, self.FArray, self.U16, self.S16, self.U16Array, self.S16Array, self.C, self.S, self.U8, self.S8, self.U8Array, self.S8Array)
}

func (self *TestTypes) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

////////////////////////////////////////////////////////////////////////////////
// String Helpers
////////////////////////////////////////////////////////////////////////////////

type Char3 [3]byte

func (chars *Char3) String() string {
	return mavlink.FixString(chars[:])
}

type Char10 [10]byte

func (chars *Char10) String() string {
	return mavlink.FixString(chars[:])
}
