/* Part of wiznet challenge
 * code is modified copy of wiznet version 1.1.0 2013/11/18
 * copyright wiznet co ltd
*/

#ifndef DHCP_H_INCLUDED
#define DHCP_H_INCLUDED


/* Retry to processing DHCP */

#define	MAX_DHCP_RETRY          2        ///< Maxium retry count
#define	DHCP_WAIT_TIME          10       ///< Wait Time 10s

/* UDP port numbers for DHCP */
#define DHCP_SERVER_PORT      	 67	      ///< DHCP server port number
#define DHCP_CLIENT_PORT         68	      ///< DHCP client port number
#define MAGIC_COOKIE             0x63825363  ///< magic number defined in RFC1048 (edwin note)

#define DCHP_HOST_NAME           "OETEL1\0"
/*
 * @brief return value of @ref DHCP_run()
 */
enum
{
   DHCP_FAILED = 0,  ///< Processing Fail
   DHCP_RUNNING=1,     ///< Processing DHCP proctocol
   DHCP_IP_ASSIGN=2,   ///< First Occupy IP from DHPC server      (if cbfunc == null, act as default default_ip_assign)
   DHCP_IP_CHANGED=3,  ///< Change IP address by new ip from DHCP (if cbfunc == null, act as default default_ip_update)
   DHCP_IP_LEASED=4,   ///< Stand by
   DHCP_STOPPED=5      ///< Stop processing DHCP protocol
};

/* DHCP state machine values. */
enum {
 STATE_DHCP_INIT         = 0,        ///< Initialize
 STATE_DHCP_DISCOVER     = 1,        ///< send DISCOVER and wait OFFER
 STATE_DHCP_REQUEST      = 2,        ///< send REQEUST and wait ACK or NACK
 STATE_DHCP_LEASED       = 3,        ///< ReceiveD ACK and IP leased
 STATE_DHCP_REREQUEST    = 4,        ///< send REQUEST for maintaining leased IP
 STATE_DHCP_RELEASE      = 5,        ///< No use
 STATE_DHCP_STOP         = 6        ///< Stop processing DHCP
};

/*
 * @brief DHCP client initialization (outside of the main loop)
 * @param s   - socket number
 * @param buf - buffer for procssing DHCP message
 * @param xid - random number for transaction
 */
void DHCP_init(uint8_t s, uint8_t * buf, uint32_t xid);
/*
 * @brief DHCP 1s Tick Timer handler
 * @note SHOULD BE register to your system 1s Tick timer handler
 */
void DHCP_time_handler(void);
/*
 * @brief Register call back function
 * @param ip_assign   - callback func when IP is assigned from DHCP server first
 * @param ip_update   - callback func when IP is changed
 * @prarm ip_conflict - callback func when the assigned IP is conflict with others.
 */

void reg_dhcp_cbfunc(void(*ip_assign)(void), void(*ip_update)(void), void(*ip_conflict)(void));
/*
 * @brief DHCP client in the main loop
 * @return    The value is as the follow \n
 *            @ref DHCP_FAILED     \n
 *            @ref DHCP_RUNNING    \n
 *            @ref DHCP_IP_ASSIGN  \n
 *            @ref DHCP_IP_CHANGED \n
 * 			  @ref DHCP_IP_LEASED  \n
 *            @ref DHCP_STOPPED    \n
 *
 * @note This function is always called by you main task.
 */
uint8_t DHCP_run(void);
/*
 * @brief Stop DHCP procssing
 * @note If you want to restart. call DHCP_init() and DHCP_run()
 */
void    DHCP_stop(void);
/* Get Network information assigned from DHCP server */
/*
 * @brief Get IP address
 * @param ip  - IP address to be returned
 */
void getIPfromDHCP(uint8_t* ip);
/*
 * @brief Get Gateway address
 * @param ip  - Gateway address to be returned
 */

void getGWfromDHCP(uint8_t* ip);
/*
 * @brief Get Subnet mask value
 * @param ip  - Subnet mask to be returned
 */

void getSNfromDHCP(uint8_t* ip);
/*
 * @brief Get DNS address
 * @param ip  - DNS address to be returned
 */
void getDNSfromDHCP(uint8_t* ip);
/*
 * @brief Get the leased time by DHCP sever
 * @retrun unit 1s
 */
uint32_t getDHCPLeasetime(void);

#endif /* DHCP_H_INCLUDED */
