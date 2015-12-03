package main

import (
	"bytes"
	"encoding/xml"
	"flag"
	"fmt"
	"go/format"
	"os"
	"os/exec"
	"path"
	"sort"
	"strconv"
	"strings"
	"text/template"

	"github.com/letalo/go-mavlink/x25"
)

var (
	protocolName   = flag.String("protocol", "all", "Protocol name. One of the XML definitions, or 'all' for all definitions.")
	definitionsDir = flag.String("defdir", "definitions", "Path of the directory with the protocol definition XML files.")
	createGo       = flag.Bool("go", true, "Create Go code.")
	goDir          = flag.String("godir", path.Join("..", "mavlink"), "Path of the directory where the Go packages will be created.")
	goFormat       = flag.Bool("gofmt", true, "Call gofmt on the resulting Go files")
	createJS       = flag.Bool("js", false, "Create ECMAScript 6 code.")
	jsDir          = flag.String("jsdir", path.Join("..", "..", "js-mavlink", "src"), "Path of the directory where the ECMAScript 6 modules will be created.")
	jsBabel        = flag.Bool("jsbabel", true, "Compile ECMAScript 6 files with babel")
	jsBabelDir     = flag.String("jsbabeldir", path.Join("..", "..", "js-mavlink", "dist"), "Ouput dir for babel compiled ECMAScript 5 files")
	print          = flag.Bool("print", false, "Print generated files to stdout.")

	funcMap = template.FuncMap{
		"UpperCamelCase": dry.StringToUpperCamelCase,
		"LowerCamelCase": dry.StringToLowerCamelCase,
	}

	generated = make(map[string]bool)

	goTemplate = template.Must(template.New("mavlink.go").Funcs(funcMap).ParseFiles(path.Join("templates", "mavlink.go")))
	jsTemplate = template.Must(template.New("mavlink.js").Funcs(funcMap).ParseFiles(path.Join("templates", "mavlink.js")))
)

func main() {
	flag.Parse()

	if *createJS && *jsBabel {
		es6filename := path.Join(*jsDir, "mavlink.js")
		es5filename := path.Join(*jsBabelDir, "mavlink.js")
		res, err := exec.Command("babel", es6filename, "--out-file", es5filename, "--source-maps", "--modules", "umd").CombinedOutput()
		if err != nil {
			panic(string(res))
		}
	}

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
			field.GoType, field.BitSize, field.ArrayLength = goType(field.CType)
			if field.ArrayLength > 0 {
				protocol.StringSizes[field.ArrayLength] = true
			}
		}

		sort.Stable(sort.Reverse(message))

		offset := 0
		for j := range message.Fields {
			field := &message.Fields[j]
			field.ByteOffset = offset
			byteSize := field.BitSize / 8
			if field.ArrayLength > 0 {
				byteSize *= field.ArrayLength
			}
			offset += byteSize
		}
	}

	generated[protocol.Name] = true

	if *createGo {
		generateGo(protocol)
	}
	if *createJS {
		generateJS(protocol)
	}
}

func generateGo(protocol *Protocol) {
	buf := bytes.NewBuffer(nil)

	err := goTemplate.Execute(buf, protocol)
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

	pkgDir := path.Join(*goDir, protocol.Name)
	filename := path.Join(pkgDir, protocol.Name+".go")

	fmt.Println("Writing", filename)

	os.Mkdir(pkgDir, 0770)

	err = dry.FileSetBytes(filename, data)
	if err != nil {
		panic(err)
	}

	if *print {
		fmt.Println(string(data))
	}
}

func generateJS(protocol *Protocol) {
	buf := bytes.NewBuffer(nil)

	err := jsTemplate.Execute(buf, protocol)
	if err != nil {
		panic(err)
	}

	data := buf.Bytes()

	filename := path.Join(*jsDir, protocol.Name+".js")

	fmt.Println("Writing", filename)

	err = dry.FileSetBytes(filename, data)
	if err != nil {
		panic(err)
	}

	if *jsBabel {
		es5filename := path.Join(*jsBabelDir, protocol.Name+".js")
		res, err := exec.Command("babel", filename, "--out-file", es5filename, "--source-maps", "--modules", "umd").CombinedOutput()
		if err != nil {
			panic(string(res))
		}
		if *print {
			data, err = dry.FileGetBytes(es5filename)
			if err != nil {
				panic(err)
			}
		}
	}

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
		bitSize := msg.Fields[i].BitSize
		if msg.Fields[i].ArrayLength > 0 {
			bitSize *= msg.Fields[i].ArrayLength
		}
		size += bitSize
	}
	return size / 8
}

func (msg *Message) CRCExtra() uint8 {
	hash := x25.NewHash()

	fmt.Fprint(hash, msg.Name+" ")
	for i := range msg.Fields {
		cType := msg.Fields[i].CType
		if cType == "uint8_t_mavlink_version" {
			cType = "uint8_t"
		}
		fmt.Fprint(hash, cType+" "+msg.Fields[i].Name+" ")
		if msg.Fields[i].ArrayLength > 0 {
			hash.WriteByte(byte(msg.Fields[i].ArrayLength))
		}
	}

	return uint8((hash.Sum & 0xFF) ^ (hash.Sum >> 8))
}

func (msg *Message) FieldsString() string {
	var buf bytes.Buffer
	buf.WriteString("fmt.Sprintf(")
	for i := range msg.Fields {
		t := msg.Fields[i].GoType
		var placeholder string
		switch {
		case strings.HasPrefix(t, "Char"):
			placeholder = `\"%s\"`
		case msg.Fields[i].ArrayLength > 0:
			placeholder = "%v"
		case strings.Contains(t, "int"), t == "byte":
			placeholder = "%d"
		case strings.Contains(t, "float"):
			placeholder = "%f"
		default:
			panic("Unknown message field type: " + t)
		}
		if i == 0 {
			buf.WriteByte('"')
		} else {
			buf.WriteByte(' ')
		}
		buf.WriteString(dry.StringToUpperCamelCase(msg.Fields[i].Name))
		buf.WriteByte('=')
		buf.WriteString(placeholder)
	}
	buf.WriteByte('"')
	for i := range msg.Fields {
		buf.WriteString(", self.")
		buf.WriteString(dry.StringToUpperCamelCase(msg.Fields[i].Name))
	}
	buf.WriteByte(')')
	return buf.String()
}

func (msg *Message) Len() int {
	return len(msg.Fields)
}

func (msg *Message) Less(i, j int) bool {
	return msg.Fields[i].BitSize < msg.Fields[j].BitSize
}

func (msg *Message) Swap(i, j int) {
	msg.Fields[i], msg.Fields[j] = msg.Fields[j], msg.Fields[i]
}

type MessageField struct {
	CType       string `xml:"type,attr"`
	Name        string `xml:"name,attr"`
	Enum        string `xml:"enum,attr"`
	Description string `xml:",innerxml"`
	GoType      string
	BitSize     int
	ArrayLength int
	ByteOffset  int
}

func (field *MessageField) IsString() bool {
	return strings.HasPrefix(field.CType, "char[")
}

func (field *MessageField) JSElementType() string {
	t := dry.StringReplaceMulti(field.CType, "_t", "", "_mavlink_version", "", "char", "uint8", "float", "float32", "double", "float64")
	if field.ArrayLength > 0 {
		t = t[:strings.IndexByte(t, '[')]
	}
	return dry.StringToUpperCamelCase(t)
}

func (field *MessageField) JSDefaultValue() string {
	switch {
	case field.IsString():
		return `""`
	case field.ArrayLength > 0:
		return fmt.Sprintf("new %sArray(%d)", field.JSElementType(), field.ArrayLength)
	case strings.HasPrefix(field.CType, "int"), strings.HasPrefix(field.CType, "uint"), field.CType == "char":
		return "0"
	case field.CType == "float", field.CType == "double":
		return "0.0"
	}
	panic("Unsupported CType: " + field.CType)
}
