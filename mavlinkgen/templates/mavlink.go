package {{.Name}}

{{$version := .Version}}

import (
	"fmt"

	"github.com/SpaceLeap/go-mavlink/mavlink"{{if .Include}}
	"github.com/SpaceLeap/go-mavlink/mavlink/{{.IncludeName}}"{{end}}
)

const (
	PROTOCOL_NAME    = "{{.Name}}"
	PROTOCOL_VERSION = {{if .Version}}{{.Version}}{{else}}0{{end}}{{if .Include}}
	PROTOCOL_INCLUDE = {{.IncludeName}}.PROTOCOL_NAME{{end}}
)

func init() {
	mavlink.ProtocolName = PROTOCOL_NAME
	mavlink.ProtocolVersion = PROTOCOL_VERSION
	{{range .Messages}}
	mavlink.MessageFactory[{{.ID}}] = func() mavlink.Message { return new({{.Name | UpperCamelCase}}) }{{end}}
}

////////////////////////////////////////////////////////////////////////////////
// Enums
////////////////////////////////////////////////////////////////////////////////

{{range .Enums}}
// {{.Name}}: {{.Description}}
const ({{range .Entries}}
	{{.Name}} = {{.Value}} // {{.Description}}{{end}}
)
{{end}}

////////////////////////////////////////////////////////////////////////////////
// Messages
////////////////////////////////////////////////////////////////////////////////

{{range .Messages}}
{{$name := .Name | UpperCamelCase}}
// {{.Description}}
type {{$name}} struct { {{range .Fields}}
	{{.Name | UpperCamelCase}} {{.GoType}} // {{.Description}}{{end}}
}

{{if eq $name "Heartbeat"}}
func NewHeartbeat() *Heartbeat {
	return &Heartbeat{MavlinkVersion: PROTOCOL_VERSION}
}
{{end}}
func (self *{{$name}}) TypeID() uint8 {
	return {{.ID}}
}

func (self *{{$name}}) TypeName() string {
	return "{{.Name}}"
}

func (self *{{$name}}) TypeSize() uint8 {
	return {{.Size}}
}

func (self *{{$name}}) TypeCRCExtra() uint8 {
	return {{.CRCExtra}}
}

func (self *{{$name}}) FieldsString() string {
	return {{.FieldsString}}
}

func (self *{{$name}}) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}
{{end}}

{{if .StringSizes}}
////////////////////////////////////////////////////////////////////////////////
// String Helpers
////////////////////////////////////////////////////////////////////////////////

func truncateZeroTerminator(chars []byte) []byte {
	for i, c := range chars {
		if c == 0 {
			return chars[:i]
		}
	}
	return chars
}

{{range $size, $_ := .StringSizes}}
type Char{{$size}}[{{$size}}]byte

func (chars *Char{{$size}}) String() string {
	return string(truncateZeroTerminator(chars[:]))
}
{{end}}

{{end}}