/*
 * Original by Wiznet, modified by .. name removed
 */

#ifndef	_DNS_H_
#define	_DNS_H_
#include <stdint.h>

/*
 * @brief Define it for Debug & Monitor DNS processing.
 * @note If defined, it depends on <stdio.h>
 */

//#define _DNS_DEBUG_
#define	MAX_DNS_BUF_SIZE	600		///< maximum size of DNS buffer mod by Edwin, ref RFC */
/*
 * @brief Maxium length of your queried Domain name
 * @todo SHOULD BE defined it equal as or greater than your Domain name length + null character(1)
 * @note SHOULD BE careful to stack overflow because it is allocated 1.5 times as MAX_DOMAIN_NAME in stack.
 * Note: name removed  fixed this, we do not use the stack anymore for this
 */

#define MAX_DOMAIN_NAME   (64) // 16       // for example "www.google.com"
#define	MAX_DNS_RETRY     2        ///< Requery Count
#define	DNS_WAIT_TIME     3        ///< Wait response time. unit 1s.

#define	IPPORT_DOMAIN     53       ///< DNS server port number

// #define DNS_MSG_ID         0x1122   ///< ID for DNS message. You can be modified it any number

/*
 * @brief DNS process initialize
 * @param s   : Socket number for DNS
 * @param buf : Buffer for DNS message
 * @param request_id : Request id used in header of message, a random value
 */
void DNS_init(uint8_t s, uint8_t * buf, uint16_t request_id);
/*
 * @brief DNS process
 * @details Send DNS query and receive DNS response
 * @param dns_ip        : DNS server ip
 * @param name          : Domain name to be queried
 * @param ip_from_dns   : IP address from DNS server
 * @return  -4 : failed. DNS server problem
 *          -1 : failed  Parse error
 *           0 : success
 * @note This funtion blocks until success or fail. max time = @ref MAX_DNS_RETRY * @ref DNS_WAIT_TIME
 */

int8_t DNS_run(const uint8_t *dns_ip, const char *name, uint8_t *ip_from_dns);
/*
 * @brief DNS 1s Tick Timer handler
 * @note SHOULD BE register to your system 1s Tick timer handler
 */

void DNS_time_handler(void);

#endif	/* _DNS_H_ */
