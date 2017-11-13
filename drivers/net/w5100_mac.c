#include <common.h>
#include <command.h>
#include <w5100_mac.h>
#include <net.h>

#ifdef CONFIG_DRIVER_W5100

//#if (CONFIG_COMMANDS & CFG_CMD_NET)

//#undef DEBUG

#define DEBUG_W5100	1

#if DEBUG_W5100
	#define DPRINTF(args...) printf(args)
#else
	#define DPRINTF(args...) { }
#endif /* DEBUG_W5100 */

/* packet page register access functions */
static void eth_reset (void)
{
	DPRINTF("[%s] W5100 reset\n",__FUNCTION__);
	/* reset NIC */
	IINCHIP_WRITE(MR, MR_RST);

	/* wait for 200ms */
	udelay (200000);
	/* Wait until the chip is reset */

}


void disp_ip(IPaddr_t ip)
{

	DPRINTF("W5100 IP : %d.%d.%d.%d\n",(int)((ip>>24)&0xFF), (int)((ip>>16)&0xFF), (int)((ip>>8)&0xFF), (int)(ip&0xFF));
}


/* Setting for use socket 0 only */
static void eth_reginit (void)
{
	u16 i;
	u8 tx_size = 0x02; 		// allocate 4KB to socket 0
	u8 rx_size = 0x02;		// allocate 4KB to socket 0

	IINCHIP_WRITE(TMSR,tx_size);
	IINCHIP_WRITE(RMSR,rx_size);


	/* interrupt for sock 0 enable */
	IINCHIP_WRITE(IMR,IM_IR_SOCK(0));


	// channel 0 : MACRAW mode
	i = 0;
MACRAW:
	if(i++ > 10000) {
		DPRINTF("MACRAW open failed...");
		return;
	}
	IINCHIP_WRITE(Sn_MR(0), Sn_MR_MACRAW); // Set MAC RAW mode for socket 0
	IINCHIP_WRITE(Sn_CR(0), Sn_CR_OPEN);	// Init or Open the socket

	udelay(10000);		// 10ms
	//DPRINTF("Sn_SSR = 0x%04x\n", IINCHIP_READ(Sn_SSR(0)));
	if((IINCHIP_READ(Sn_SSR(0)) & 0xff) != SOCK_MACRAW) {
		IINCHIP_WRITE(Sn_CR(0), Sn_CR_CLOSE);
		goto MACRAW;
	}

	DPRINTF("[%s] - W5100 register init\n",__FUNCTION__);
}

void eth_halt (void)
{
	/* disable transmitter/receiver mode */
	//IINCHIP_WRITE(MR, 0xf900);

	/* I'm not sure this is right - no reference found */	
	IINCHIP_WRITE(Sn_CR(0), Sn_CR_CLOSE);
	IINCHIP_WRITE(Sn_MR(0), Sn_MR_CLOSE);


	DPRINTF("[%s] - W5100 halt\n",__FUNCTION__);
	
	/* "shutdown" to show ChipID or kernel wouldn't find he cs8900 ... */
}

int eth_init (bd_t * bd)
{
	IPaddr_t ip;

	eth_reset();

	/* MR, IMR, ICFGR */
	IINCHIP_WRITE(MR, 0x00);

	IINCHIP_WRITE(IMR, 0x0);	// no interrupt


	/* set the ethernet address */
	if (getenv ("ethaddr")) {
		uchar enetaddr[6];
		eth_getenv_enetaddr("ethaddr", enetaddr);

		printf("W5100 MAC Address : %02X:%02X:%02X:%02X:%02X:%02X\n",
			enetaddr[0],enetaddr[1],enetaddr[2],enetaddr[3],enetaddr[4],enetaddr[5] );

		IINCHIP_WRITE(SHAR0,enetaddr[0]);
		IINCHIP_WRITE(SHAR1,enetaddr[1]);
		IINCHIP_WRITE(SHAR2,enetaddr[2]);
		IINCHIP_WRITE(SHAR3,enetaddr[3]);
		IINCHIP_WRITE(SHAR4,enetaddr[4]);
		IINCHIP_WRITE(SHAR5,enetaddr[5]);

	}
	else
	{
		printf("%s - Can not find ethaddr in env!!\n",__FUNCTION__);
		return 0;
	}


	/* set the IP,Gateway,Subnet */
	ip = bd->bi_ip_addr;
	ip = ntohl(ip);
	
	//display sip to lcd
	//disp_ip(ip);
	
	
	IINCHIP_WRITE(SIPR0, (ip>>24)&(0xFF));
	IINCHIP_WRITE(SIPR1, (ip>>16)&(0xFF));
	IINCHIP_WRITE(SIPR2, (ip>>8)&(0xFF));
	IINCHIP_WRITE(SIPR3, ip&0xff);


	DPRINTF("W5100 IP : %d.%d.%d.%d\n",(int)((ip>>24)&0xFF), (int)((ip>>16)&0xFF), (int)((ip>>8)&0xFF), (int)(ip&0xFF));

	
	ip = getenv_IPaddr ("netmask");
	ip = ntohl(ip);
	
//	IINCHIP_WRITE(SUBR, ip>>16);
 //   IINCHIP_WRITE(SUBR2, ip&0xffff);

	IINCHIP_WRITE(SUBR0, (ip>>24)&(0xFF));
	IINCHIP_WRITE(SUBR1, (ip>>16)&(0xFF));
	IINCHIP_WRITE(SUBR2, (ip>>8)&(0xFF));
	IINCHIP_WRITE(SUBR3, ip&0xff);
	

	DPRINTF("W5100 NETMASK : %d.%d.%d.%d\n",(int)((ip>>24)&0xFF), (int)((ip>>16)&0xFF), (int)((ip>>8)&0xFF), (int)(ip&0xFF));



	ip = getenv_IPaddr ("gatewayip");
	ip = ntohl(ip);
//	IINCHIP_WRITE(GAR, ip>>16);
//	IINCHIP_WRITE(GAR2, ip&0xffff);

	IINCHIP_WRITE(GAR0, (ip>>24)&(0xFF));
	IINCHIP_WRITE(GAR1, (ip>>16)&(0xFF));
	IINCHIP_WRITE(GAR2, (ip>>8)&(0xFF));
	IINCHIP_WRITE(GAR3, ip&0xff);
	DPRINTF("W5100 GATEWAY IP : %d.%d.%d.%d\n",(int)((ip>>24)&0xFF), (int)((ip>>16)&0xFF), (int)((ip>>8)&0xFF), (int)(ip&0xFF));
	
	eth_reginit ();
	return 0;
}

/* Get a data block via Ethernet */
int eth_rx (void)
{
	u16 data_size = 0;	// In MAC RAW mode, data header means the size of the data

	u16 overflow_upper_len = 0;
	u16 rest_len = 0;

	u16 loc_info = 0;
	u16 sock_received_size = 0;
	u16 sock_rx_offset = 0;

	u8* sock_addr_to_read = NULL; 
	u8* packet_buf_addr = NULL;

	// check received data length
	sock_received_size = IINCHIP_READ(Sn_RX_RSR(0)); // Received data size register - indicates the size of the data received
	sock_received_size = (sock_received_size << 8)|(IINCHIP_READ(Sn_RX_RSR1(0)));


	if(sock_received_size == 0)
	{
		 return 0;
	}
	else
	{
		DPRINTF("[%s] - rx size : %d\n",__FUNCTION__,sock_received_size);
	}

	// location info to read the data	
	loc_info = IINCHIP_READ(Sn_RX_RD(0));
	loc_info = (loc_info<<8)|(IINCHIP_READ(Sn_RX_RD1(0)));

	DPRINTF("[%s] - offset : %d\n", __FUNCTION__, loc_info);


	sock_rx_offset = loc_info & RX_MEM_MASK;
	sock_addr_to_read = (u8*)(sock_rx_offset + RX_MEM_BASE_ADDR); // Calculate the address to read

	packet_buf_addr = (u8*)NetRxPackets[0];

	
	/* Getting Header */
	if((sock_rx_offset + RX_PKT_HEADER_SIZE) > (RX_MEM_MASK + 1))
	{
		/* When overflow with the header in socket RX memory */
		overflow_upper_len = ((RX_MEM_MASK + 1) - sock_rx_offset);

		if(overflow_upper_len > 0)
		{
			memcpy(&data_size , sock_addr_to_read , overflow_upper_len);
		}

		rest_len = RX_PKT_HEADER_SIZE - overflow_upper_len;

		memcpy((u8*)((&data_size)+overflow_upper_len) ,(u8*)RX_MEM_BASE_ADDR, rest_len);
		
		sock_rx_offset = rest_len;

	}
	else
	{

		memcpy((u8*)(&data_size) , sock_addr_to_read , RX_PKT_HEADER_SIZE);
		sock_rx_offset += RX_PKT_HEADER_SIZE;
	}	

	data_size = ntohs(data_size);
	DPRINTF("[%s] - data size : %d\n",__FUNCTION__, data_size);

	/* Getting Data*/
	sock_addr_to_read = (u8*)(RX_MEM_BASE_ADDR + sock_rx_offset); // Calculate the address to read

	if((sock_rx_offset + data_size) > (RX_MEM_SIZE + 1))
	{
		/* When overflow with the data in socket RX memory */
		overflow_upper_len = ((RX_MEM_MASK + 1) - sock_rx_offset);

		if(overflow_upper_len > 0)
		{
			memcpy(packet_buf_addr , sock_addr_to_read , overflow_upper_len);
		}

		rest_len = data_size - overflow_upper_len;

		memcpy((u8*)(packet_buf_addr+overflow_upper_len) ,(u8*)RX_MEM_BASE_ADDR, rest_len);
	

	}
	else
	{
		memcpy(packet_buf_addr , sock_addr_to_read , data_size);
	}


	loc_info = loc_info + data_size + RX_PKT_HEADER_SIZE;

	IINCHIP_WRITE(Sn_RX_RD(0),(loc_info>>8)&(0xFF));
	IINCHIP_WRITE(Sn_RX_RD1(0), loc_info & 0xFF );
	IINCHIP_WRITE(Sn_CR(0), Sn_CR_RECV);

	/* Pass the packet up to the protocol layers. */
	NetReceive (NetRxPackets[0], data_size);

	return data_size;
}

/* Send a data block via Ethernet. */
int eth_send (volatile void *packet, int length)
{

	volatile int tmo, tmp;
	volatile unsigned char  cnt = 0;

	u16 s;
	u16 free_tx_size = 0;

	u16 overflow_upper_len = 0;
	u16 rest_len = 0;

	u16 loc_info = 0;
	u16 sock_tx_size = 0;
	u16 sock_tx_offset = 0;

	u8* sock_addr_to_send = NULL; 

	int i = 0;

	free_tx_size = IINCHIP_READ(Sn_TX_FSR(0));
	free_tx_size = (free_tx_size << 8) + IINCHIP_READ(Sn_TX_FSR1(0));
	if(free_tx_size < length) {
		DPRINTF("Not enough free size.\n");
		eth_reset();
		eth_reginit();
		return 0;
	}

	if( (length < 0) || (length > 65535) )
		DPRINTF("eth_send:exceed length!\n");


	loc_info = IINCHIP_READ(Sn_TX_WR(0));
	loc_info = (loc_info<<8)|(IINCHIP_READ(Sn_TX_WR1(0)));

//	loc_info = ntohs(loc_info);
	DPRINTF("[%s] - offset : %d\n",__FUNCTION__, loc_info);


	sock_tx_offset = loc_info & TX_MEM_MASK;
	sock_addr_to_send = (u8*)(TX_MEM_BASE_ADDR + sock_tx_offset); // Calculate the address to read


	DPRINTF("[%s] ",__FUNCTION__);
	for(i=0; i<length; i++)
	{
		DPRINTF("%02X ",*(((unsigned char*)packet)+i));
	}
	DPRINTF("\n");


	if((sock_tx_offset + length) > (TX_MEM_SIZE + 1))
	{
		/* When overflow with the data in socket TX memory */
		overflow_upper_len = ((TX_MEM_MASK + 1) - sock_tx_offset);

		if(overflow_upper_len > 0)
		{
			memcpy(sock_addr_to_send, packet , overflow_upper_len);
		}

		rest_len = length - overflow_upper_len;

		memcpy((u8*)TX_MEM_BASE_ADDR ,(((u8*)packet)+overflow_upper_len) ,rest_len);
	

	}
	else
	{
		memcpy(sock_addr_to_send, packet, length);
	}


	loc_info = loc_info + length;

	IINCHIP_WRITE(Sn_TX_WR(0),(loc_info>>8)&(0xFF));
	IINCHIP_WRITE(Sn_TX_WR1(0), loc_info & 0xFF );

	IINCHIP_WRITE(Sn_CR(0), Sn_CR_SEND);

	/* wait for transfer to succeed */
	tmo = get_timer (0) + 5 * CONFIG_SYS_HZ;
//	tmo = get_timer (0) + 1000000 * CONFIG_SYS_HZ;

	//DPRINTF("[%s] set time out [%d]\n",__FUNCTION__,tmo);
	while ((s = IINCHIP_READ(Sn_IR(0)) & Sn_IR_SEND_OK) != Sn_IR_SEND_OK) {
		
		cnt++;

		if ((tmp = get_timer (0)) >= tmo)
		{

			DPRINTF("[%s] timeout[%d][%d]\n",__FUNCTION__,cnt,tmp);
			return 0;
		}
	}

	IINCHIP_WRITE(Sn_IR(0), Sn_IR_SEND_OK);

	DPRINTF("[%s] send ok.\n",__FUNCTION__);
	return 0;
}

//#endif	/* COMMANDS & CFG_NET */

#endif	/* CONFIG_DRIVER_W5100 */
