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

	"github.com/SpaceLeap/go-mavlink/x25"
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
		message.Name = dry.StringToUpperCamelCase(message.CName)
		message.Description = strings.Replace(message.Description, "\n", "\n// ", -1)
		// message.Description = strings.Replace(message.Description, "\t", "", -1)
		for j := range message.Fields {
			field := &message.Fields[j]
			field.Name = dry.StringToUpperCamelCase(field.CName)
			field.Description = strings.Replace(field.Description, "\n", " ", -1)
			field.Type = dry.StringReplaceMulti(field.CType,
				"_t", "",
				"_mavlink_version", "",
				"char", "byte",
				"float", "float32",
				"double", "float64")
			if index := strings.IndexByte(field.Type, '['); index != -1 {
				field.arrayLength, _ = strconv.Atoi(field.Type[index+1 : len(field.Type)-1])
				field.Type = field.Type[index:] + field.Type[:index]
			}
			if strings.HasSuffix(field.Type, "byte") {
				field.bitSize = 8
				if field.arrayLength > 0 {
					protocol.StringSizes[field.arrayLength] = true
					field.Type = fmt.Sprintf("Char%d", field.arrayLength)
				}
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
	CName       string         `xml:"name,attr"`
	Description string         `xml:"description"`
	Fields      []MessageField `xml:"field"`
}

func (msg *Message) Size() (size int) {
	for i := range msg.Fields {
		size += msg.Fields[i].bitSize
	}
	return size / 8
}

func (msg *Message) CRCExtra() uint8 {
	hash := x25.NewHash()

	fmt.Fprint(hash, msg.CName+" ")
	for i := range msg.Fields {
		fmt.Fprint(hash, msg.Fields[i].CType+" "+msg.Fields[i].CName+" ")
		if msg.Fields[i].arrayLength > 0 {
			hash.WriteByte(byte(msg.Fields[i].arrayLength))
		}
	}

	return uint8((hash.Sum() & 0xFF) ^ (hash.Sum() >> 8))
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
	Type        string
	CType       string `xml:"type,attr"`
	Name        string
	CName       string `xml:"name,attr"`
	Description string `xml:",innerxml"`
	bitSize     int
	arrayLength int
}
