/* Hash.h
 * Basic interface for MD5 hashes
 */

#include <stdint.h>

#pragma once

namespace hebi::firmware {

class Hash {
public:
    static const uint8_t MD5_SUM_LEN = 16;
    static const uint8_t MD5_SUM_STR_LEN = (2 * MD5_SUM_LEN) + 1;

    static char nibble2hex(uint8_t in){
        static const char* digits = "0123456789ABCDEF";
        return digits[in & 0x0F];
    }

   /* getStringHash
    * Converts a md5 hash into the nice readable hex format from the raw buffer.
    */
    static void getStringHash(const unsigned char* src, char* buf) {
        for (uint8_t i = 0; i < MD5_SUM_LEN; ++i){
            buf[2*i]     = nibble2hex(src[i] >> 8);
            buf[2*i + 1] = nibble2hex(src[i]);
        }
    }
};

} /* hebi::firmware */

