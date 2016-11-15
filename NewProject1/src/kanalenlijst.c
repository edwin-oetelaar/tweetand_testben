/* kanalenlijst
 by Edwin van den Oetelaar
 4 december 2014
 GPL license etc blah blah
*/
#include <stddef.h>
#include "term_io.h"
#include "kanalenlijst.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"
#include "event_groups.h"
#include "semphr.h"

static const struct channel channels[] = {
    {
        .text = "Radio Arrow 1",
        .host = NULL,
        .ip =
        {91,221,151,156},
        .port = 80,
        .mount = "",
        .mode = pm_listening
    },
    {
        .text = "Radio Arrow 2",
        .host = NULL,
        .ip =
        {91,221,151,178},
        .port = 9109,
        .mount = "",
        .mode = pm_listening
    },
    {
        .text = "Test Channel1 RX",  /* luisteren naar eigen uitzendingen */
        .host = "s1.streamsolution.nl",
        .ip = {0, 0, 0, 0},
        .port = 8000,
        .mount = "test",
        .mode = pm_listening
    },
    {
        .text = "Test Channel2 RX",  /* luisteren naar eigen uitzendingen */
        .host = "s1.streamsolution.nl",
        .ip = {0, 0, 0, 0},
        .port = 8000,
        .mount = "test2",
        .mode = pm_listening
    },
    {
        .text = "Trans Chan Test ",
        .host = "s1.vergadering-gemist.nl",
        .port = 8000,
        .mount = "test",
        .passw = "test",
        .mode = pm_sending
    },
    {
        .text = "Trans Chan Test2",
        .host = "s1.vergadering-gemist.nl",
        .port = 8000,
        .mount = "test2",
        .passw = "test",
        .mode = pm_sending
    },
    {
        .text = "Radio 1 AAC 32kb",
        .host = "icecast.omroep.nl",
        .port = 80,
        .mount = "radio1-sb-aac",
        .mode = pm_listening
    },
    {
        .text = "Radio 2 AAC 32kb",
        .host = "icecast.omroep.nl",
        .port = 80,
        .mount = "radio2-sb-aac",
        .mode = pm_listening
    },
    {
        .text = "Radio 3 AAC 32kb",
        .host = "icecast.omroep.nl",
        .port = 80,
        .mount = "3fm-sb-aac",
        .mode = pm_listening
    },
    {
        .text = "Radio 4 AAC 32kb",
        .host = "icecast.omroep.nl",
        .port = 80,
        .mount = "radio4-sb-aac",
        .mode = pm_listening
    },
    {
        .text = "Radio 5 AAC 32kb",
        .host = "icecast.omroep.nl",
        .port = 80,
        .mount = "radio5-sb-aac",
        .mode = pm_listening
    },
    {
        .text = "Radio 6 AAC 32kb",
        .host = "icecast.omroep.nl",
        .port = 80,
        .mount = "radio6-sb-aac",
        .mode = pm_listening
    },
    {
        .text = "BNR nieuwsradio",
        .host = "icecast-bnr.cdp.triple-it.nl",
        .port = 80,
        .mount = "bnr_aac_32_04",
        .mode = pm_listening
    },
    {
        .text = "Absolute Radio UK",
        .host = "aacplus-ac-32.timlradio.co.uk",
        .port = 80,
        .mount = "/",
        .mode = pm_listening
    }
};

/* return number of items in the kanalenlijst */
uint32_t kl_get_count(void)
{
    return sizeof(channels) / sizeof(channels[0]);
}

/* get pointer to kanaal data struct by index (0 based) */
const struct channel *kl_get_channel(uint32_t index)
{

    if (index < kl_get_count()) {
        return &channels[index];
    }
    xprintf("assert: kl_get_channel i=%ld\r\n",index);
    return NULL;
}
