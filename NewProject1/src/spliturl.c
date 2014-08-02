/* split url functies
 * nodig voor radio/blindenradio
 */
/* made by Edwin van den Oetelaar */

// #include "wiztypes.h"

/* Scheduler includes */
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
#include <stdlib.h>
#include <limits.h>
#include "spliturl.h"

#define DEBUG 1

#ifdef DEBUG
#include <stdio.h>
#endif

http_req_t *http_req_init(void)
{
    http_req_t *r = pvPortMalloc(sizeof(http_req_t)); // calloc(sizeof(http_req_t), 1);
    memset(r, 0, sizeof(http_req_t));
    // add sane defaults
    r->port = 80;
    return r;
}
static void clean_http_req(http_req_t *req)
{
    if (req) {
        vPortFree(req->abs_path);
        vPortFree(req->authority);
        vPortFree(req->cookie);
        vPortFree(req->fragment);
        vPortFree(req->hostname);
        vPortFree(req->parameter);
        vPortFree(req->password);
        vPortFree(req->pvalue);
        vPortFree(req->query);
        vPortFree(req->username);
        memset(req, 0, sizeof(*req));
        req->port = 80;
    }
}

void http_req_destroy(http_req_t *r)
{
    clean_http_req(r);
    vPortFree(r);
}

void http_req_dump(http_req_t *r)
{
    xprintf("Dump of Request2 type:\n");
    xprintf("Scheme    : '%s'\n", r->scheme == 1 ? "HTTP" : "unsupported");
    xprintf("Authority : '%s'\n", r->authority);
    xprintf("Username  : '%s'\n", r->username);
    xprintf("Password  : '%s'\n", r->password);
    xprintf("Hostname  : '%s'\n", r->hostname);
    xprintf("Port      : '%d'\n", r->port);
    xprintf("Abs Path  : '%s'\n", r->abs_path);
    xprintf("Query     : '%s'\n", r->query);
    xprintf("Fragment  : '%s'\n", r->fragment);
}

/*
 [scheme:][//authority][path][?query][#fragment]
 where the characters :, /, ?, and # stand for themselves.
 The scheme-specific part of a hierarchical URI consists of the characters between the scheme and fragment components.
 The authority component of a hierarchical URI is, if specified, either server-based or registry-based.
 A server-based authority parses according to the familiar syntax
 [user-info@]host[:port]
 return : 0 on success
 not 0 on error
 */

uint8_t http_req_from_url(const char *url, http_req_t *req)
{

    // clean old memory when we get called
    clean_http_req(req);

    const uint8_t schemelen = 7;
    if (strncasecmp("http://", url, schemelen)) {
        // not found http scheme
        req->scheme = -1;
        return 100;
    }

    // scheme found
    req->scheme = 1;

    const char *ptrFirstAuthority = url + schemelen;
    const char *ptrPastUrl = url + strlen(url);
    // zoek einde van authority
    const char *p; // tmp pointer somewhere into url
    p = ptrFirstAuthority;

    while (p < ptrPastUrl) {
        if (*p == '/' || *p == '?' || *p == '#')
            break;
        p++;
    }

    const char *ptrPastAuthority = p;

    size_t auth_len = ptrPastAuthority - ptrFirstAuthority;

    if ((req->authority = pvPortMalloc(auth_len + 1))) {
        strncpy(req->authority, ptrFirstAuthority, auth_len);
        req->authority[auth_len] = 0;
    } else {
        // no memory
        req->scheme = -1;
        return 1;
    }

    // split into [user [:pass] @ ] hostname [: portname]
    const char * const ptrA = memchr(ptrFirstAuthority, '@', auth_len);

    // flag if @ char was found
    uint8_t bHaveAtChar = ptrA ? 1 : 0;

    // @ niet gevonden => hostname starts with first char of Authority
    // @ wel gevonden => skip over @ char
    const char *ptrH = bHaveAtChar ? ptrA + 1 : ptrFirstAuthority;

    // find end of hostname inside Authority, but past the @ char
    const char *ptrPastHostname = ptrH;
    while (ptrPastHostname < ptrPastAuthority) {
        if (*ptrPastHostname == ':')
            break;
        ptrPastHostname++;
    }
    // post: ptrB points after hostname
    // calc hostname length
    int hostlen = ptrPastHostname - ptrH;

    // not sane, then exit
    if ((hostlen == 0) || (hostlen > 255)) {
        req->scheme = -1;
        return 101;
    }

    // sane hostname, copy it
    if ((req->hostname = pvPortMalloc(hostlen + 1))) {
        strncpy(req->hostname, ptrH, hostlen);
        req->hostname[hostlen] = 0;
    } else {
        // no memory
        req->scheme = -1;
        return 2;
    }

    // handle portnumber
    if (*ptrPastHostname == ':') {
        char pnum[7]; // some space
        // char *pnum = pvPortMalloc(16);
        // if (pnum) {
        int i = 0;
        // zolang het een '0'..'9' is dan is het een poortnummer, maximaal 5 chars 1..65535
        p = ptrPastHostname + 1; // skip over ':'
        while ((*p >= '0') && (*p <= '9') // is a digit
                && (p < ptrPastAuthority) // range check
                && (i < 6) // range check number of digits
              ) {
            pnum[i] = *p;
            p++;
            i++;
        }
        pnum[i] = 0; // string end
        // number given use it, empty uses default
        if (i)
            req->port = atoi(pnum);
        // }
    }

    // handle user and password
    if (bHaveAtChar) {
        // we have @ sign
        // user:pass is before @ sign
        // pre: ptrA points to @ sign
        // pre: ptrFirstAuthority points to first char of authority
        p = ptrFirstAuthority;
        while (p < ptrA) {
            if (*p == ':')
                break;
            p++;
        }
        // post: p points to '@' or ':'
        int userlen = p - ptrFirstAuthority;

        if (userlen && (userlen < 255)) {
            if ((req->username = pvPortMalloc(userlen + 1))) {
                strncpy(req->username, ptrFirstAuthority, userlen);
                req->username[userlen] = 0;
            } else {
                // no memory
                req->scheme = -1;
                return 4;
            }
        }
        // check if we have password
        if (*p == ':') {
            p++; // skip the ':'
            int paslen = ptrA - p;
            if (paslen && (paslen < 255)) {
                if ((req->password = pvPortMalloc(paslen + 1))) {
                    strncpy(req->password, p, paslen);
                    req->password[paslen] = 0;
                } else {
                    // no memory
                    req->scheme = -1;
                    return 3;
                }
            }
        }
    }
    // we zoeken nu de pathname
    // die zit [scheme] authority [ path ] [? query]  [ # fragment ]
    // pathname start na authority en loopt tot .. ? of #

    p = ptrPastAuthority;
    int pathlen = 0;
    while (p < ptrPastUrl) {
        if (*p == '?' || *p == '#')
            break;
        p++;
        pathlen++;
    }

    if (pathlen && (pathlen < 255)) {
        if ((req->abs_path = pvPortMalloc(pathlen + 1))) {
            strncpy(req->abs_path, ptrPastAuthority, pathlen);
            req->abs_path[pathlen] = 0;
        } else {
            req->scheme = -1;
            return 5;
        }
    } else {
        // empty or invalid path, default to '/'
        if ((req->abs_path = pvPortMalloc(2))) {
            req->abs_path[0] = '/';
            req->abs_path[1] = 0;
        } else {
            req->scheme = -1;
            return 5;
        }
    }

    // handle query, p points to char after pathname
    if (*p == '?') {
        // query ? found
        const char *ptrPastQmark = p + 1;
        // find end of string or # char
        while (p < ptrPastUrl) {
            if (*p == '#')
                break;
            p++;
        }
        // post : p now points to '#' or 0x00, needed for fragment handling
        int querylen = p - ptrPastQmark;
        // found a query if sane copy it
        if (querylen && (querylen < 255)) {
            if ((req->query = pvPortMalloc(querylen + 1))) {
                strncpy(req->query, ptrPastQmark, querylen);
                req->query[querylen] = 0;
            } else {
                req->scheme = -1;
                return 6;
            }
        }
    }

    // handle fragment
    if (*p == '#') {
        const char *ptrPastHash = p + 1;
        int fraglen = ptrPastUrl - ptrPastHash;
        // found fragment, if sane copy it
        if (fraglen && (fraglen < 255)) {
            if ((req->fragment = pvPortMalloc(fraglen + 1))) {
                strncpy(req->fragment, ptrPastHash, fraglen);
                req->fragment[fraglen] = 0;
            } else {
                req->scheme = -1;
                return 7;
            }
        }
    }

    return 0; // no errors
}

// decode base64

static const char b64[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
int nbytes[] = { 3, 1, 1, 2 };

uint8_t xlate(unsigned char *in, int phase, uint8_t *buf)
{
    unsigned char out[3];
    out[0] = in[0] << 2 | in[1] >> 4;
    out[1] = in[1] << 4 | in[2] >> 2;
    out[2] = in[2] << 6 | in[3] >> 0;

    int i;

    for (i = 0; i < nbytes[phase]; i++)    {
        buf[i] = out[i];
    }
    return nbytes[phase];
}

/* name : base64decode (in,len,out,len)
 * convert base64encoded buffer to binary data
 * return : length of binary data
 */
size_t base64decode(const char *input, /* input buffer*/
                    size_t inputlen, /* length of input */
                    uint8_t *output, /* output buf */
                    size_t outputlen /* length of output buf*/
                   )
{

    int c, phase;
    unsigned char in[4];
    char *p;
    size_t i = 0;
    size_t i_output = 0;
    phase = 0;
    while (i < inputlen && ((i_output + 4) < outputlen)) {
        c = input[i];
        if (c == '=') {
            i_output += xlate(in, phase, output + i_output);
            break;
        }
        p = strchr(b64, c);
        if (p) {
            in[phase] = p - b64;
            phase++;
            if (phase == 4)
                phase = 0;

            if (phase == 0) {
                i_output += xlate(in, phase, output + i_output);
                in[0] = in[1] = in[2] = in[3] = 0;
            }
        }
        i++;
    }
    return i_output;
}
/* name : base64encode(in,len,out,len)
 * convert binary data to base64
 * return 1 on success 0==fail
 */
int base64encode(const void* data_buf, // input buf
                 size_t dataLength, // input size
                 char* result, // output buf
                 size_t resultSize // output size
                )
{
    const uint8_t *data = data_buf;
    size_t resultIndex = 0;
    size_t x;
    uint32_t n = 0;
    int padCount = dataLength % 3;
    uint8_t n0, n1, n2, n3;

    /* increment over the length of the string, three characters at a time */
    for (x = 0; x < dataLength; x += 3) {
        /* these three 8-bit (ASCII) characters become one 24-bit number */
        n = data[x] << 16;

        if ((x + 1) < dataLength)
            n += data[x + 1] << 8;

        if ((x + 2) < dataLength)
            n += data[x + 2];

        /* this 24-bit number gets separated into four 6-bit numbers */
        n0 = (uint8_t) (n >> 18) & 63;
        n1 = (uint8_t) (n >> 12) & 63;
        n2 = (uint8_t) (n >> 6) & 63;
        n3 = (uint8_t) n & 63;

        /*
         * if we have one byte available, then its encoding is spread
         * out over two characters
         */
        if (resultIndex >= resultSize)
            return 0; /* indicate failure: buffer too small */
        result[resultIndex++] = b64[n0];
        if (resultIndex >= resultSize)
            return 0; /* indicate failure: buffer too small */
        result[resultIndex++] = b64[n1];

        /*
         * if we have only two bytes available, then their encoding is
         * spread out over three chars
         */
        if ((x + 1) < dataLength) {
            if (resultIndex >= resultSize)
                return 0; /* indicate failure: buffer too small */
            result[resultIndex++] = b64[n2];
        }

        /*
         * if we have all three bytes available, then their encoding is spread
         * out over four characters
         */
        if ((x + 2) < dataLength) {
            if (resultIndex >= resultSize)
                return 0; /* indicate failure: buffer too small */
            result[resultIndex++] = b64[n3];
        }
    }

    /*
     * create and add padding that is required if we did not have a multiple of 3
     * number of characters available
     */
    if (padCount > 0) {
        for (; padCount < 3; padCount++) {
            if (resultIndex >= resultSize)
                return 0; /* indicate failure: buffer too small */
            result[resultIndex++] = '=';
        }
    }
    if (resultIndex >= resultSize)
        return 0; /* indicate failure: buffer too small */
    result[resultIndex] = 0;
    return 1; /* indicate success */
}

// #define UNITTESTSPLITURL
#if 0
uint8_t parse_header_line(conn_state_t *cs, const char *line, const uint8_t len)
{
    char tmp[255];
    uint16 i;
    uint16 j;
    char *p;
    uint8 rv = 0;

    if (!cs->http_response && (len > 6)) {
        /* handle response header HTTP/1.? 200 OK
         * or ICY 200 OK
         */

        /* forget/reset all stuff about the request */
        cs->cstate = 0;
        /* match HTTP/1.0 200 or ICY 200 */
        i = sscanf(line, "%s %d", tmp, &j);
        /* i = number of conversions, should be 2 */
        /* accept http or icy resonse */
        if (i == 2 && ((tmp[0] == 'h' && tmp[1] == 't' && tmp[2] == 't'
                        && tmp[3] == 'p') || (tmp[0] == 'i' && tmp[1] == 'c' && tmp[2]
                                              == 'y'))) {
            /* store response code 200/302/404 etc */
            cs->http_response = j;
            /* store response type http/icy, set stream flag */
            //  if ((tmp[0] == 'i')) printf("\r\nICY");
            cs->cstate |= (tmp[0] == 'i') ? (CS_ICY | CS_STREAM) : CS_HTTP;
        } else {
            printf("\r\nj != 2 or !ICY !HTTP ");
            /* response header is bad, return 1 */
            rv = 1;
        }
    } else {
        /* handle normal headers, we had a response code, Key-AAP : Value */
        /* if it starts with 32 or 9 it is continuation of previous line, ignore for now */
        if (line[0] == ' ' || line[0] == '\t') {
            /* ignore line */
            //  printf("\r\nIgnore continuation");
        } else {
            /* break the line, parse fields */
            p = strchr(line, ':');
            if (p) {
                *p = '\0';

                printf("\r\nkey=%s", line);
                /* skip over whitespace after : , fixes Content-type:[\s*]html/bull */
                while (isspace(*p) || *p == '\0')
                    p++;

                printf("\r\nvalue=%s", p);
                if (!strcmp(line, "content-type")) {
                    printf("\r\nhandle content-type");
                    /* find out what this content-type means, could be DATA/or Playlist 1/2, or bullshit */
                    if (strstr(p, "audio")) {
                        printf("\r\nhandle audio");
                        /* audio seen */
                        if (strstr(p, "mp3") || strstr(p, "mpeg") || strstr(p,
                                "mpegaudio") || strstr(p, "mpg")) {
                            cs->cstate |= CS_DATA | CS_MP3;
                        }
                        if (strstr(p, "aac") || strstr(p, "3gpp") || strstr(p,
                                "x-wav")) {
                            cs->cstate |= CS_DATA | CS_AAC;
                        }
                        if (strstr(p, "url") && strstr(p, "mpeg")) {
                            cs->cstate |= CS_PLAYLIST | CS_M3U;
                            cs->cstate &= ~CS_DATA; /* unset data flag */

                        }
                        if (strstr(p, "scpls") || strstr(p, "/pls")) {
                            cs->cstate |= CS_PLAYLIST | CS_PLS;
                        }
                        if (strstr(p, "ogg")) {
                            cs->cstate |= CS_DATA | CS_OGG;
                        }
                        if (strstr(p, "ms-wma") || strstr(p, "ms-asf")) {
                            cs->cstate |= CS_DATA | CS_WMA;
                        }
                        if (strstr(p, "mid")) {
                            cs->cstate |= CS_DATA | CS_MIDI;
                        }
                    } else if (strstr(p, "application")) {
                        /* some types are audio
                         *
                         */
                        if (strstr(p, "/ogg") || strstr(p, "/x-ogg")) {
                            cs->cstate |= CS_DATA | CS_WMA;
                        }

                    } else if (strstr(p, "text")) {
                        /* text/html */
                    }

                    printf("\r\npls=%d", (uint16) cs->cstate & CS_PLS);
                    printf("\r\nm3u=%d", (uint16) cs->cstate & CS_M3U);
                    printf("\r\nplaylist=%d", (uint16) cs->cstate & CS_PLAYLIST);
                    printf("\r\ndata=%d", (uint16) cs->cstate & CS_DATA);
                    printf("\r\nmp3=%d", (uint16) cs->cstate & CS_MP3);
                    printf("\r\naac=%d", (uint16) cs->cstate & CS_AAC);
                    printf("\r\nwma=%d", (uint16) cs->cstate & CS_WMA);
                    printf("\r\nogg=%d", (uint16) cs->cstate & CS_OGG);
                    printf("\r\nis_midi=%d", (uint16) cs->cstate & CS_MIDI);
                    printf("\r\nis_icy=%d", (uint16) cs->cstate & CS_ICY);
                    printf("\r\nis_http=%d", (uint16) cs->cstate & CS_HTTP);
                    printf("\r\nhttp rc=%d", (uint16) cs->http_response);
                } else if (!strcmp(line, "location")) {
                    printf("\r\nhandle redirect");
                    if (cs->http_response > 299 && cs->http_response < 308) {
                        /* looks like redirect */
                        /* we must store the value for new connection */
                        printf("\r\nRedirect to: %s", p);
                    }
                } else if (!strcmp(line, "accept-ranges")) {
                    printf("\r\nhandle ranges");
                    cs->cstate &= ~CS_STREAM; /* unset stream flag */
                } else if (!strcmp(line, "content-length")) {
                    cs->cstate &= ~CS_STREAM; /* unset stream flag */
                    printf("\r\nhandle content length : is_stream=%d",
                           (uint16) cs->cstate & CS_STREAM);
                    // cs->cstate &= ~CS_STREAM; /* unset stream flag */

                } else if (!strcmp(line, "icy-metaint")) {
                    sscanf(p, "%d", &cs->meta_interval);

                    printf("\r\nhandle meta-interval (%s) (%d)", p,
                           cs->meta_interval);
                    cs->cstate |= CS_STREAM;

                } else if (!strcmp(line, "icy-pub")) {
                    printf("\r\nhandle icy-pub");
                    cs->cstate |= CS_STREAM;
                } else if (!strcmp(line, "icy-url")) {
                    printf("\r\nhandle icy-url");
                    cs->cstate |= CS_STREAM;
                } else if (!strcmp(line, "icy-br")) {
                    printf("\r\nhandle icy-br");
                    cs->cstate |= CS_STREAM;
                } else if (!strcmp(line, "icy-name")) {
                    printf("\r\nhandle icy-name");
                    cs->cstate |= CS_STREAM;
                } else if (!strcmp(line, "icy-genre")) {
                    printf("\r\nhandle icy-genre");
                } else if (!strcmp(line, "location")) {
                    printf("\r\nhandle location redirect");
                }
            } else {
                printf("\r\n: not found");
            }
        }
    }
    return rv;
}
#endif

#ifdef UNITTESTSPLITURL
int main(int argc, char **argv)
{

    char
    *urls[] = {
        "http://edwin:password@oetelaar.com/dit_is_een/test/met///zut?kut&peren=%77aap#zutzut/&*(/",
        "http://edwin:@w.p/%20/%20/moeilijk.php?kut&peren=%77aap#zutzut/&*(/",
        "http://edwin:@w.p/%20/%20/moeilijk.php?#",
        "http://a::@w.p/%20/%20/moeilijk.php?kut&peren=%77aap#zutzut/&*(/",
        "http://oetelaar.com:/kut?a#b",
        "http://oetelaar.com:99/kut#", "http://a@oetelaar.com:#",
        "http://a:@s:77#f", 0
    };

    int i = 0;
    while (urls[i] != 0) {
        printf("doe %s\n", urls[i]);
        http_req_t *r = http_req_init();

        if (!http_req_from_url(urls[i], r)) {
            http_req_dump(r);
        } else {
            printf(" failed\n");
        }
        http_req_destroy(r);
        i++;
    }
    return 0;
}
#endif
