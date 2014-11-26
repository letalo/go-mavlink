package main

import (
	"bytes"
	"encoding/xml"
	"flag"
	"fmt"
	"go/format"
	"strconv"
	"strings"
	"text/template"

	"github.com/ungerik/go-dry"
)

type Protocol struct {
	Name        string
	StringSizes map[int]bool

	XMLName  xml.Name  `xml:"mavlink"`
	Version  string    `xml:"version"`
	Enums    []Enum    `xml:"enums>enum"`
	Messages []Message `xml:"messages>message"`
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
	Name        string         `xml:"name,attr"`
	Description string         `xml:"description"`
	Fields      []MessageField `xml:"field"`
	Size        int
}

type MessageField struct {
	Type        string `xml:"type,attr"`
	Name        string `xml:"name,attr"`
	Description string `xml:",innerxml"`
}

func main() {
	var (
		protocol Protocol
		goFormat bool
	)
	flag.StringVar(&protocol.Name, "proto", "common", "Protocol name. One of the XML definitions.")
	flag.BoolVar(&goFormat, "fmt", true, "Call go fmt on the result file")
	flag.Parse()

	protocol.StringSizes = make(map[int]bool)

	err := dry.FileUnmarshallXML(fmt.Sprintf("definitions/%s.xml", protocol.Name), &protocol)
	if err != nil {
		panic(err)
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
		message.Name = dry.StringToUpperCamelCase(message.Name)
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
				"float", "float32")
			if index := strings.IndexByte(field.Type, '['); index != -1 {
				field.Type = field.Type[index:] + field.Type[:index]
			}
			if strings.HasSuffix(field.Type, "]byte") {
				size := dry.StringToInt(field.Type[1 : len(field.Type)-len("]byte")])
				protocol.StringSizes[size] = true
				field.Type = "Char" + strconv.Itoa(size)
			}
		}
	}

	templ := template.Must(template.ParseFiles("go.template"))

	// fmt.Printf("%#v\n", &protocol)

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

	goFilename := fmt.Sprintf("../mavlink/%s/%s.go", protocol.Name, protocol.Name)

	err = dry.FileSetBytes(goFilename, data)
	if err != nil {
		panic(err)
	}

	fmt.Println(string(data))

	// file, err := os.Create(goFilename)
	// defer file.Close()

	// file.Close()

	// err = exec.Command("go", "fmt", goFilename).Run()
	// if err != nil {
	// 	panic(err)
	// }

	// err = templ.Execute(os.Stdout, &protocol)
	// if err != nil {
	// 	panic(err)
	// }
}
