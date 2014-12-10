package mavlink

func FixString(chars []byte) string {
	for i, c := range chars {
		switch c {
		case '\n', '\r':
			chars[i] = ' '
		case 0:
			return string(chars[:i])
		}
	}
	return string(chars)
}
