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
	constructor() { {{range .Fields}}
		this.{{.Name | LowerCamelCase}} = {{if eq .CType "uint8_t_mavlink_version"}}PROTOCOL_VERSION{{else}}{{.JSDefaultValue}}{{end}}; // {{.Description}}{{end}}
	}

	get typeID() {
		return {{.ID}};
	}

	get typeName() {
		return "{{.Name}}";
	}

	get typeSize() {
		return {{.Size}};
	}

	get typeCRCExtra() {
		return {{.CRCExtra}};
	}

	fieldsString() {
		return `{{range $i, $val := .Fields}}{{if gt $i 0}} {{end}}{{$val.Name | LowerCamelCase}}=${this.{{$val.Name | LowerCamelCase}}}{{end}}`;
	}
}
{{end}}