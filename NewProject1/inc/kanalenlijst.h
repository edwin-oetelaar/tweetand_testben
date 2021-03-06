//
// Created by oetelaar on 3/16/16.
//

#ifndef NEWPROJECT1_KANALENLIJST_H_H
#define NEWPROJECT1_KANALENLIJST_H_H

#include <stdint.h>

enum {
    pm_listening = 0, pm_sending = 1, pm_parrot=2
};

struct channel {
    char *text;
    char *host;
    char *passw;
    uint8_t ip[4];
    uint16_t port;
    char *mount;
    char mode; //  = pm_listening
} ;


const struct channel *kl_get_channel(uint32_t index);
uint32_t kl_get_count(void);

#endif //NEWPROJECT1_KANALENLIJST_H_H
