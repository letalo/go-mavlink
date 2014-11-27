// MAVLINK_CRC_EXTRA MAVLINK_MESSAGE_CRCS
// Field reordering
// Message size calc
// def message_checksum(msg):
//     '''calculate a 8-bit checksum of the key fields of a message, so we
//        can detect incompatible XML changes'''
//     crc = mavutil.x25crc(msg.name + ' ')
//     for f in msg.ordered_fields:
//         crc.accumulate(f.type + ' ')
//         crc.accumulate(f.name + ' ')
//         if f.array_length:
//             crc.accumulate(chr(f.array_length))
//     return (crc.crc&0xFF) ^ (crc.crc>>8) 

package main

import (
	"bytes"
	"encoding/xml"
	"flag"
	"fmt"
	"go/format"
	"os"
	"path/filepath"
	"sort"
	"strconv"
	"strings"
	"text/template"

	"github.com/ungerik/go-dry"
)

var (
	goFormat bool

	templ = template.Must(template.ParseFiles("go.template"))
)

func main() {
	var protocol Protocol

	flag.StringVar(&protocol.Name, "protocol", "common", "Protocol name. One of the XML definitions.")
	flag.BoolVar(&goFormat, "fmt", true, "Call go fmt on the result file")
	flag.Parse()

	generate(&protocol)
}

func generate(protocol *Protocol) {
	protocol.StringSizes = make(map[int]bool)

	xmlFilename := fmt.Sprintf("definitions/%s.xml", protocol.Name)

	fmt.Println("Parsing", xmlFilename)

	err := dry.FileUnmarshallXML(xmlFilename, &protocol)
	if err != nil {
		panic(err)
	}

	if protocol.Include != "" {
		protocol.IncludedProtocol = &Protocol{Name: protocol.IncludeName()}
		generate(protocol.IncludedProtocol)
	}

	for i := range protocol.Enums {
		enum := &protocol.Enums[i]
		enum.Description = strings.Replace(enum.Description, "\n", "\n// ", -1)
		// enum.Description = strings.Replace(enum.Description, "\t", "", -1)
		for j := range enum.Entries {
			enum.Entries[j].Description = strings.Replace(enum.Entries[j].Description, "\n", " ", -1)
		}
	}

	for i := range protocol.Messages {
		message := &protocol.Messages[i]
		message.Name = dry.StringToUpperCamelCase(message.NameUpper)
		message.Description = strings.Replace(message.Description, "\n", "\n// ", -1)
		// message.Description = strings.Replace(message.Description, "\t", "", -1)
		for j := range message.Fields {
			field := &message.Fields[j]
			field.Name = dry.StringToUpperCamelCase(field.Name)
			field.Description = strings.Replace(field.Description, "\n", " ", -1)
			field.Type = dry.StringReplaceMulti(field.Type,
				"_t", "",
				"_mavlink_version", "",
				"char", "byte",
				"float", "float32",
				"double", "float64")
			if index := strings.IndexByte(field.Type, '['); index != -1 {
				field.Type = field.Type[index:] + field.Type[:index]
			}
			if strings.HasSuffix(field.Type, "]byte") {
				size := dry.StringToInt(field.Type[1 : len(field.Type)-len("]byte")])
				protocol.StringSizes[size] = true
				field.Type = "Char" + strconv.Itoa(size)
				field.bitSize = 8
			} else {
				t := field.Type[strings.IndexByte(field.Type, ']')+1:]
				if sizeStart := strings.IndexAny(t, "8136"); sizeStart != -1 {
					field.bitSize, _ = strconv.Atoi(t[sizeStart:])
				}
				if field.bitSize == 0 {
					panic("Unknown message field size")
				}
			}
		}
		sort.Stable(message)
	}

	buf := bytes.NewBuffer(nil)

	err = templ.Execute(buf, &protocol)
	if err != nil {
		panic(err)
	}

	data := buf.Bytes()
	if goFormat {
		data, err = format.Source(data)
		if err != nil {
			panic(err)
		}
	}

	// todo path.Join
	pkgDir, _ := filepath.Abs(fmt.Sprintf("../mavlink/%s", protocol.Name))
	goFilename := fmt.Sprintf("%s/%s.go", pkgDir, protocol.Name)

	fmt.Println("Writing", goFilename)

	os.Mkdir(pkgDir, 0770)

	err = dry.FileSetBytes(goFilename, data)
	if err != nil {
		panic(err)
	}

	// fmt.Println(string(data))
}

type Protocol struct {
	Name             string
	StringSizes      map[int]bool
	IncludedProtocol *Protocol

	XMLName  xml.Name  `xml:"mavlink"`
	Version  string    `xml:"version"`
	Include  string    `xml:"include"`
	Enums    []Enum    `xml:"enums>enum"`
	Messages []Message `xml:"messages>message"`
}

func (protocol *Protocol) IncludeName() string {
	return strings.TrimSuffix(strings.ToLower(protocol.Include), ".xml")
}

type Enum struct {
	Name        string      `xml:"name,attr"`
	Description string      `xml:"description"`
	Entries     []EnumEntry `xml:"entry"`
}

type EnumEntry struct {
	Value       uint8            `xml:"value,attr"`
	Name        string           `xml:"name,attr"`
	Description string           `xml:"description"`
	Params      []EnumEntryParam `xml:"param"`
}

type EnumEntryParam struct {
	Index       uint8  `xml:"index,attr"`
	Description string `xml:",innerxml"`
}

type Message struct {
	ID          uint8          `xml:"id,attr"`
	Name        string         `xml:"-"`
	NameUpper   string         `xml:"name,attr"`
	Description string         `xml:"description"`
	Fields      []MessageField `xml:"field"`
	Size        int
}

func (msg *Message) Len() int {
	return len(msg.Fields)
}

func (msg *Message) Less(i, j int) bool {
	return msg.Fields[i].bitSize >= msg.Fields[j].bitSize
}

func (msg *Message) Swap(i, j int) {
	msg.Fields[i], msg.Fields[j] = msg.Fields[j], msg.Fields[i]
}

type MessageField struct {
	Type        string `xml:"type,attr"`
	Name        string `xml:"name,attr"`
	Description string `xml:",innerxml"`
	bitSize     int
}

func (field *MessageField) BitSize() int {
	return 0
}
