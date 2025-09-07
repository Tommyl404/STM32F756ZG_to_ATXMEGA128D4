#include "hal.h"
#include "simpleserial.h"
#include "aes.h"
#include <string.h>

static uint8_t key[16];
static uint8_t out[16];

uint8_t handle_key(uint8_t *k, uint8_t len){
    if(len != 16) return 0x01;
    memcpy(key, k, 16);
    return 0x00;
}

uint8_t handle_pt(uint8_t *pt, uint8_t len){
    if(len != 16) return 0x01;
    trigger_high();
    AES128_ECB_encrypt(pt, key, out);
    trigger_low();
    simpleserial_put('r', 16, out);
    return 0x00;
}

uint8_t handle_echo(uint8_t *in, uint8_t len){
    simpleserial_put('t', len, in);
    return 0x00;
}

int main(void){
    platform_init();
    init_uart();
    trigger_setup();
    simpleserial_init();
    simpleserial_addcmd('k', 16, handle_key);
    simpleserial_addcmd('p', 16, handle_pt);
    simpleserial_addcmd('t', 0, handle_echo);

    while(1){
        simpleserial_get();
    }
}
