package x25

/*
#import <stdint.h>

void crc_accumulate(uint8_t data, uint16_t *crcAccum)
{
        uint8_t tmp;

        tmp = data ^ (uint8_t)(*crcAccum &0xff);
        tmp ^= (tmp<<4);
        *crcAccum = (*crcAccum>>8) ^ (tmp<<8) ^ (tmp <<3) ^ (tmp>>4);
}
*/
import "C"

// C reference implementation to test against
func crc_accumulate(data uint8, crcAccum *uint16) {
	C.crc_accumulate(C.uint8_t(data), (*C.uint16_t)(crcAccum))
}
