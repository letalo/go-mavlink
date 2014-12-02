package main

import (
	"bytes"
	"encoding/xml"
	"flag"
	"fmt"
	"go/format"
	"os"
	"path"
	"sort"
	"strconv"
	"strings"
	"text/template"

	"github.com/SpaceLeap/go-mavlink/x25"
	"github.com/ungerik/go-dry"
)

var (
	protocolName   = flag.String("protocol", "all", "Protocol name. One of the XML definitions, or 'all' for all definitions.")
	definitionsDir = flag.String("defdir", "definitions", "Path of the directory with the protocol definition XML files.")
	goDir          = flag.String("godir", path.Join("..", "mavlink"), "Path of the directory where the Go packages will be created.")
	goFormat       = flag.Bool("go_fmt", true, "Call go fmt on the result file")
	print          = flag.Bool("print", false, "Print generated files to stdout")

	funcMap = template.FuncMap{
		"UpperCamelCase": dry.StringToUpperCamelCase,
		"LowerCamelCase": dry.StringToLowerCamelCase,
	}

	generated = make(map[string]bool)

	templ = template.Must(template.New("go.template").Funcs(funcMap).ParseFiles("go.template"))
)

func main() {
	flag.Parse()

	if *protocolName != "all" {
		generate(*protocolName)
	} else {
		files, err := dry.ListDirFiles(*definitionsDir)
		if err != nil {
			panic(err)
		}
		for _, filename := range files {
			if !strings.Contains(filename, "_test") {
				generate(strings.TrimSuffix(filename, ".xml"))
			}
		}
	}
}

func generate(name string) {
	if generated[name] {
		return
	}

	protocol := &Protocol{
		Name:        name,
		StringSizes: make(map[int]bool),
	}

	xmlFilename := path.Join(*definitionsDir, protocol.Name+".xml")

	fmt.Println("Parsing", xmlFilename)

	err := dry.FileUnmarshallXML(xmlFilename, &protocol)
	if err != nil {
		panic(err)
	}

	if protocol.Include != "" {
		generate(protocol.IncludeName())
	}

	for i := range protocol.Enums {
		enum := &protocol.Enums[i]
		enum.Description = strings.Replace(enum.Description, "\n", "\n// ", -1)
		for j := range enum.Entries {
			if enum.Entries[j].Value == nil {
				enum.Entries[j].Value = new(uint8)
				*enum.Entries[j].Value = uint8(j)
			}
			enum.Entries[j].Description = strings.Replace(enum.Entries[j].Description, "\n", " ", -1)
		}
	}

	for i := range protocol.Messages {
		message := &protocol.Messages[i]
		message.Description = strings.Replace(message.Description, "\n", "\n// ", -1)
		for j := range message.Fields {
			field := &message.Fields[j]
			field.Description = strings.Replace(field.Description, "\n", " ", -1)
			field.GoType, field.bitSize, field.arrayLength = goType(field.CType)
			if field.arrayLength > 0 {
				protocol.StringSizes[field.arrayLength] = true
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
	if *goFormat {
		data, err = format.Source(data)
		if err != nil {
			panic(err)
		}
	}

	// todo path.Join
	pkgDir := path.Join(*goDir, protocol.Name)
	goFilename := path.Join(pkgDir, protocol.Name+".go")

	fmt.Println("Writing", goFilename)

	os.Mkdir(pkgDir, 0770)

	err = dry.FileSetBytes(goFilename, data)
	if err != nil {
		panic(err)
	}

	generated[protocol.Name] = true

	if *print {
		fmt.Println(string(data))
	}
}

func goType(cType string) (name string, bitSize int, arrayLength int) {
	name = dry.StringReplaceMulti(cType,
		"_t", "",
		"_mavlink_version", "",
		"char", "byte",
		"float", "float32",
		"double", "float64")
	if index := strings.IndexByte(name, '['); index != -1 {
		arrayLength, _ = strconv.Atoi(name[index+1 : len(name)-1])
		name = name[index:] + name[:index]
	}
	if strings.HasSuffix(name, "byte") {
		bitSize = 8
		if arrayLength > 0 {
			name = fmt.Sprintf("Char%d", arrayLength)
		}
	} else {
		t := name[strings.IndexByte(name, ']')+1:]
		if sizeStart := strings.IndexAny(t, "8136"); sizeStart != -1 {
			bitSize, _ = strconv.Atoi(t[sizeStart:])
		}
		if bitSize == 0 {
			panic("Unknown message field size")
		}
	}
	return name, bitSize, arrayLength
}

type Protocol struct {
	Name        string
	StringSizes map[int]bool

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
	Value       *uint8           `xml:"value,attr"` // ptr to make optional
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
	Name        string         `xml:"name,attr"`
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

	fmt.Fprint(hash, msg.Name+" ")
	for i := range msg.Fields {
		fmt.Fprint(hash, msg.Fields[i].CType+" "+msg.Fields[i].Name+" ")
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
	Name        string `xml:"name,attr"`
	Description string `xml:",innerxml"`
	GoType      string
	bitSize     int
	arrayLength int
}
