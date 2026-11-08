#include "as5048a.h"
#include <stddef.h>
static uint16_t next, sent;
uint16_t encoder_spi_transfer16(int index, uint16_t tx) { (void)index; sent=tx; return next; }
void *memset(void *p, int c, size_t n) { unsigned char *b=p; while(n--) *b++=(unsigned char)c; return p; }
#define CHECK(x) do {if(!(x)) return __LINE__;} while(0)
int main(void)
{
    as5048a_encoder_init(0);
    next=0x0003; /* valid but not yet an angle response */
    as5048a_encoder_read_single_turn(0);
    CHECK(sent==0xffff && !as5048a_encoder_get_pos_valid(0));
    next=0x8001; /* even parity, angle 1 */
    as5048a_encoder_read_single_turn(0);
    CHECK(as5048a_encoder_get_pos_valid(0) && as5048a_encoder_get_abs(0)==1);
    next=0x8000; /* timeout / bad parity */
    for(int i=0;i<4;++i) as5048a_encoder_read_single_turn(0);
    CHECK(!as5048a_encoder_get_pos_valid(0) && as5048a_encoder_get_wdg_error(0));
    CHECK(as5048a_encoder_get_abs(0)==1);
    next=0x0003;
    as5048a_encoder_read_single_turn(0);
    CHECK(as5048a_encoder_get_pos_valid(0) && !as5048a_encoder_get_wdg_error(0));
    next=0xc000; /* valid parity, EF */
    as5048a_encoder_read_single_turn(0);
    CHECK(as5048a_encoder_get_error(0) && !as5048a_encoder_get_pos_valid(0));
    CHECK(as5048a_encoder_get_abs(0)==3);
    as5048a_encoder_reset_error(0);
    CHECK(sent==0xffff && as5048a_encoer[0].last_error_flags==0);
    CHECK(!as5048a_encoder_get_pos_valid(0));
    next=0x0003; as5048a_encoder_read_single_turn(0);
    CHECK(as5048a_encoder_get_pos_valid(0) && !as5048a_encoder_get_error(0));
    return 0;
}
