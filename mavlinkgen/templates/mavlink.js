import * as mavlink from "mavlink";
{{if .Include}}import * as {{.IncludeName}} from "{{.IncludeName}}";{{end}}

export const PROTOCOL_NAME = "{{.Name}}";
{{if .Version}}export const PROTOCOL_VERSION = {{.Version}};{{end}}{{if .Include}}
export const PROTOCOL_INCLUDE = {{.IncludeName}}.PROTOCOL_NAME;{{end}}

export function init() {
	{{if .Include}}{{.IncludeName}}.init(){{else}}for (let [i, _] of mavlink.MessageFactory) mavlink.MessageFactory[i] = null;{{end}}

	mavlink.ProtocolName = PROTOCOL_NAME{{if .Version}}
	mavlink.ProtocolVersion = PROTOCOL_VERSION{{end}}
	
	{{range .Messages}}
	mavlink.MessageFactory[{{.ID}}] = () => new {{.Name | UpperCamelCase}}();{{end}}
}

////////////////////////////////////////////////////////////////////////////////
// Enums
////////////////////////////////////////////////////////////////////////////////

{{range .Enums}}
// {{.Name}}: {{.Description}}{{range .Entries}}
export const {{.Name}} = {{.Value}}; // {{.Description}}{{end}}
{{end}}

////////////////////////////////////////////////////////////////////////////////
// Messages
////////////////////////////////////////////////////////////////////////////////

{{range .Messages}}
{{$name := .Name | UpperCamelCase}}
// {{.Description}}
class {{$name}} extends mavlink.Message {
	constructor(buffer = new ArrayBuffer({{.Size}})) {
		this.data = new DataView(buffer);{{range .Fields}}{{if eq .CType "uint8_t_mavlink_version"}}
		this.{{.Name | LowerCamelCase}} = PROTOCOL_VERSION;{{end}}{{end}}
	}

	static get typeID() {
		return {{.ID}};
	}

	static get typeName() {
		return "{{.Name}}";
	}

	static get typeSize() {
		return {{.Size}};
	}

	static get typeCRCExtra() {
		return {{.CRCExtra}};
	}

	fieldsString() {
		return `{{range $i, $val := .Fields}}{{if gt $i 0}} {{end}}{{$val.Name | LowerCamelCase}}=${this.{{$val.Name | LowerCamelCase}}}{{end}}`;
	}

	{{range .Fields}}
	{{if .IsString}}
	// {{.Description}}
	get {{.Name | LowerCamelCase}}() {
		return new DataView(this.data.buffer, {{.ByteOffset}}, {{.ArrayLength}}).getStringNT(0);
	}

	// {{.Description}}
	set {{.Name | LowerCamelCase}}(value) {
		return new DataView(this.data.buffer, {{.ByteOffset}}, {{.ArrayLength}}).setStringNT(0, value);
	}
	{{else if .ArrayLength}}
	// {{.Description}}
	get{{.Name | UpperCamelCase}}(index) {
		return this.data.get{{.JSElementType}}({{.ByteOffset}} + {{.BitSize}}/8*index{{if ne .BitSize 8}}, true{{end}});
	}

	// {{.Description}}
	set{{.Name | UpperCamelCase}}(index, value) {
		return this.data.set{{.JSElementType}}({{.ByteOffset}} + {{.BitSize}}/8*index, value{{if ne .BitSize 8}}, true{{end}});
	}
	{{else}}
	// {{.Description}}
	get {{.Name | LowerCamelCase}}() {
		return this.data.get{{.JSElementType}}({{.ByteOffset}}{{if ne .BitSize 8}}, true{{end}});
	}

	// {{.Description}}
	set {{.Name | LowerCamelCase}}(value) {
		return this.data.set{{.JSElementType}}({{.ByteOffset}}, value{{if ne .BitSize 8}}, true{{end}});
	}
	{{end}}
	{{end}}
}
{{end}}