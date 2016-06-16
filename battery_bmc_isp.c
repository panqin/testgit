/******************************************************************************
 *File_name	:	battery_bmc_isp.c
 *Contains 	:	SSU2125 Battery ISP
 *
 *Author	:liuyuanyang 
 *Time		:2014.06.26
 *Copyright	:MacroSAN Co.inc(2014) all right reserved
 ******************************************************************************/
 
#include <stdlib.h>
#include <stdio.h>
#include <stddef.h>
#include <string.h>
#include "osf.h"
#include "ses.h"
#include "sxp.h"
#include "flm.h"
#include "sxp_hw.h"
#include "uart_hw.h"
#include "uart_regs.h"
#include "uart.h"
#include "cpld.h"
#include "ecmr.h"
#include "cicint.h"
#include "m34khal.h"
#include "pmcfw_types.h"
#include "macrosan_debug.h"
#include "battery_bmc_isp.h"
#include "inner_com_transfer.h"

/*LPC2132 on-chip flash partation
**********************************************************************************************************************
**Sector 	0          1         2          3          4          5          6           7          8       		 *
**Addr  0000-0FFF  1000-1FFF 2000-2FFF  3000-3FFF  4000-4FFF  5000-5FFF  6000-6FFF   7000-7FFF  8000-FFFF			 *
**Type		4K         4K        4K         4K         4K         4K         4K          4K         32K   			 *
**********************************************************************************************************************/

/*LPC2138 on-chip flash partation
**********************************************************************************************************************
**Sector	0          1         2          3          4          5          6           7          8          9 ... *
**Addr	0000-0FFF  1000-1FFF 2000-2FFF  3000-3FFF  4000-4FFF  5000-5FFF  6000-6FFF   7000-7FFF  8000-FFFF  8000-FFFF *
**Type		4K         4K        4K         4K         4K         4K         4K          4K         32K   	   32K...*
**********************************************************************************************************************/
EXTERN void active_event_insert(UINT8 module_id, UINT8 event_id, UINT8 data0, UINT8 data1);
EXTERN void  disk_poweroff_soon(UINT32 map);
EXTERN UINT32 odsp_resvered_disks_map;

PUBLIC BOOL battery_upgrade_is_approved = FALSE;
PUBLIC BOOL ac_down_event_has_occurred = FALSE;
PUBLIC UINT16 thread_id_bmc_isp = 0;
PUBLIC UINT8 local_bat_upgrade_status = 0;
PUBLIC UINT8 opper_bat_upgrade_status = 0;

PRIVATE CHAR chipflag;
PRIVATE UINT8 bat_to_upgrade = 0;
PRIVATE bat_upgrade_log_struct bat_upgrade_log[BATTERY_UPGRADE_LOG_MAX_NUM];
PRIVATE battery_bmc_isp_task_parms_struct *battery_bmc_isp_task_parms_ptr = NULL;

/*Logger*/
PRIVATE void bat_log_insert(UINT16 data)
{
	static UINT8 index = 0;
	static UINT8 time = 0;

	bat_upgrade_log[index].num = bat_to_upgrade;
	bat_upgrade_log[index].time = time;
	bat_upgrade_log[index].addr = data;

	index++;
	time++;

	if(BATTERY_UPGRADE_LOG_MAX_NUM == index)
	{
		index = 0;
	}
}

PRIVATE void bat_log_clean(void)
{
	memset((UINT8 *)bat_upgrade_log, 0, sizeof(bat_upgrade_log_struct) * BATTERY_UPGRADE_LOG_MAX_NUM);
}

/*
 *@uart_id	:Which UART to use
 *@sentbuf	:Point to the command which will be transfered
 *@rcvbuf	:Container of the response which for command in "sentbuf"
 *
 *												lyy # 2014.06.26
 */
PRIVATE CHAR cmd_communication(UINT8 uart_id, CHAR *sentbuf, CHAR *rcvbuf)
{
	UINT32 sent_len = 0, rcv_len = 0, real_rcv_len = 0;
	CHAR real_rcv_buf[100];
	
	memset(real_rcv_buf, 0, 100);
	/* Send command */
	sent_len = strlen(sentbuf);
	if(PMC_SUCCESS != uart_tx_mult(uart_id, sentbuf, sent_len))
	{
		if((0x3F == sentbuf[0]) && (1 == sent_len))
		{
			/* Send error in "?" */
			return SENT_UPDATE_CMD_ERR;
		}
		/* Delete "\r\n" */
		sentbuf[sent_len - 2] = '\0';
		return SENT_UPDATE_CMD_ERR;
	}

	/* Receive Response */
	rcv_len = strlen(rcvbuf);
	if(strcmp(sentbuf, "E 0 0\r\n") == 0) /* Erase command special */
	{
		osf_thread_sleep(osf_time_ms_to_ticks(1000));
	}

	real_rcv_len = uart_rx_mult(uart_id, real_rcv_buf, rcv_len);
	if(real_rcv_len < rcv_len)
	{		
		return RECEIVEE_CMD_ACK_TIMOUT;
	}
	
	real_rcv_buf[real_rcv_len] = '\0';
	/*Read Chip ID Proc Alone*/
	if(0 == strcmp(sentbuf, "J\r\n"))
	{
		if(0 == strcmp(real_rcv_buf, "J\r\n0\r\n196369\r\n"))
		{
			chipflag = CHIPLPC2132;
			return 0;
		}
		else if(0 == strcmp(real_rcv_buf, "J\r\n0\r\n196389\r\n"))
		{
			chipflag = CHIPLPC2138;
			return 0;			
		}
		else if(0 == strcmp(real_rcv_buf, "J\r0\r\n637615927\r\n"))
		{
			chipflag = CHIPLPC1768;
			return 0;			
		}
		else
		{
			return RECEIVEE_CMD_UNMATCH;	
		}
	}
	else
	{
		if(0 != strcmp(real_rcv_buf, rcvbuf))
		{
			return RECEIVEE_CMD_UNMATCH;		
		}
		
		return 0;
	}
}

/*
 *@uart_id	:Which UART to use
 *@databuf	:Pointer of the data which want to transfer
 *@len		:Data length of transfer
 *
 *										lyy # 2014.06.26
 */
PRIVATE UINT8 send_data_to_flash(UINT8 uart_id, CHAR *databuf, UINT32 len)
{	
	UINT32	i = 0, rcv_len = 0;
	CHAR	rcv_buf[100];
	memset(rcv_buf, 0, 100);

	if(PMC_SUCCESS != uart_tx_mult(uart_id, databuf, len))
	{
		return SENT_ENCODE_DATA_ERR;
	}

	/*Receive Response*/
	rcv_len = uart_rx_mult(uart_id, rcv_buf, len);
	if(rcv_len < len)
	{
		return RECEIVEE_CMD_ACK_TIMOUT;
	}
	
	rcv_buf[rcv_len] = '\0';
	for(i = 0; i < len; i++)
	{
		if(rcv_buf[i] != databuf[i])
		{
			return RECEIVEE_ACK_DATA_UNMATCH;
		}
	}

	return 0;
}

/*
 *ISP main
 *
 *@uart_id	:Which UART to use
 *@buf		:Pointer of the data which want to transfer
 *@len		:Data length of transfer
 *
 *										lyy # 2014.06.26
 */
PRIVATE UINT8 program_flash(UINT8 uart_id, UCHAR *buf, UINT32 len) 
{
	CHAR 	sent_str[100], ask_str[100], out_buf[256];
	UINT32	data_len;
	FLOAT64	tmp;
	UCHAR	block_num, data_count = 0, *buf_ptr;
	INT		line_len = 0, line_cnt = 0, str_len = 0;
	INT		sect = 0, sect_tmp=0, out_pos = 0;
	ULONG	in_pos = 0, crc = 0;
	
 	data_len = len;
	buf_ptr = buf;

	memset(sent_str, 0, 100);
	memset(ask_str,  0, 100);	
	memset(out_buf,  0, 256);	
	
	/*Send "?" and waiting for response*/
	sent_str[0] = 0x3F;
	sent_str[1] = '\0';
	strcpy(ask_str, "Synchronized\r\n");

	if(cmd_communication(uart_id, sent_str, ask_str))
	{
		bat_log_insert(__LINE__);
		return CMD_SEND_NO_RECIVE_ACK;
	}
	DBG_ISP_SPL("\nSynchronizing with jmc/bcb..\r");
	bat_log_insert(__LINE__);
	
	/*
	 *Host sent sync word("Synchronized\r\n"),
	 *Receive Response("Synchronized<CR><LF>OK<CR><LF>")
	 */
	strcpy(sent_str, "Synchronized\r\n");
	strcpy(ask_str, "Synchronized\r\nOK\r\n");
	
	if(0 != cmd_communication(uart_id, sent_str, ask_str))
	{
		bat_log_insert(__LINE__);
		return CMD_SEND_AND_RECIVE_CHECK_ERR;
	}
	DBG_ISP_SPL("Synchronizing with jmc/bcb...\r");
	bat_log_insert(__LINE__);
	
	/*
	 *Send The oscillator frequency 25MHz,(25000<CR><LF>)
	 *Receive Response(25000<CR><LF>OK<CR><LF>)
	 */
	strcpy(sent_str, "25000\r\n");
	strcpy(ask_str, "25000\r\nOK\r\n");
	
	if(0 != cmd_communication(uart_id, sent_str, ask_str))
	{
		bat_log_insert(__LINE__);
		return CMD_SEND_AND_RECIVE_CHECK_ERR;
	}
	DBG_ISP_SPL("Synchronizing with jmc/bcb....\r");
	bat_log_insert(__LINE__);
	
	/*
	 *Send "U 23130" to Unlock,(U 23130<CR><LF>)
	 *Receive Response (U 23130<CR><LF>0<CR><LF>)
	 */
	strcpy(sent_str, "U 23130\r\n");
	strcpy(ask_str, "U 23130\r\n0\r\n");
	
	if(0 != cmd_communication(uart_id, sent_str, ask_str))
	{
		bat_log_insert(__LINE__);
		return CMD_SEND_AND_RECIVE_CHECK_ERR;
	}
	DBG_ISP_SPL("Synchronizing with jmc/bcb.....\r");
	bat_log_insert(__LINE__);
	
	/*
	 *Send "J" to read chip id,(J<CR><LF>)
	 *Receive Response(J<CR><LF>0<CR><LF>X<CR><LF>)
	 */
	strcpy(sent_str, "J\r\n");
	strcpy(ask_str, "J\r\n0\r\n196369\r\n");	
	if(0 != cmd_communication(uart_id, sent_str, ask_str))
	{
		bat_log_insert(__LINE__);
		return CMD_SEND_AND_RECIVE_CHECK_ERR;
	}
	
	/*lpc2132 max inbound flash size 64k,lpc2138 is 512k*/
	if(CHIPLPC2132 == chipflag)
	{
		if((data_len/1024) > 64)
		{
			bat_log_insert(__LINE__);
			return UPDATE_FILE_OVER_FLASH_VOLUME;
		}
	}
	else if(CHIPLPC2138 == chipflag)
	{
		if((data_len/1024) > 512)
		{
			bat_log_insert(__LINE__);
			return UPDATE_FILE_OVER_FLASH_VOLUME;
		}
	}
	else if(CHIPLPC1768 == chipflag)
	{
		if((data_len/1024) > 512)
		{
			bat_log_insert(__LINE__);
			return UPDATE_FILE_OVER_FLASH_VOLUME;
		}
	}

	DBG_ISP_SPL("Synchronizing with jmc/bcb......\r");
	bat_log_insert(__LINE__);
	
	/*
	 *Send FLASH to write,(P 0 X<CR><LF>)
	 *Receive Response (P 0 X<CR><LF>0<CR><LF>)
	 */
	if(CHIPLPC2132 == chipflag)
	{
		block_num = (data_len)/4096;	/*0-7 is 4k£¬8 is 32k*/
		if(block_num > 7)
		{
			block_num = 8;
		}
	}
	else if(CHIPLPC2138 == chipflag)
	{
		block_num = (data_len)/4096;	/*0-7 is 4k£¬8 is 32k, 9 is 64k ...*/
		if((block_num > 7) && (block_num < 125))
		{
			if(block_num < 120)
			{
				block_num = block_num/8 + 7;
			}
			else
			{
				block_num = block_num - 120 + 22;
			}
		}
	}
	else if(CHIPLPC1768 == chipflag)
	{
		block_num = (data_len)/4096;	/*0-15 is 4k£¬16 is 32k,17 is 32k...*/
		if((block_num > 15) && (block_num < 126))
		{
			if(block_num < 120)
			{
				block_num = block_num/8 + 14;
			}
			else
			{
				block_num = block_num - 120 + 29;
			}
		}
	}

	DBG_ISP_SPL("Synchronizing with jmc/bcb.......\r");

	sprintf(sent_str, "P 0 %d\r\n", block_num);
	strcpy(ask_str, sent_str);	
	if(CHIPLPC1768 == chipflag)
	{
		ask_str[strlen(ask_str) - 1] = 0;
	}
	
	strcat(ask_str, "0\r\n");
	if(0 != cmd_communication(uart_id, sent_str, ask_str))
	{
		bat_log_insert(__LINE__);
		return CMD_SEND_AND_RECIVE_CHECK_ERR;
	}
	DBG_ISP_SPL("Synchronizing with jmc/bcb........\r");
	bat_log_insert(__LINE__);
	
	/*
	 *Send erase FLASH,(E 0 X<CR><LF>)
	 *Receive Response(E 0 X<CR><LF>0<CR><LF>)
	 */
	sprintf(sent_str, "E 0 %d\r\n", block_num);
	strcpy(ask_str,sent_str);
	if(CHIPLPC1768 == chipflag)
	{
		ask_str[strlen(ask_str) - 1] = 0;
	}
	strcat(ask_str,"0\r\n");

	if(0 != cmd_communication(uart_id, sent_str, ask_str))
	{
		bat_log_insert(__LINE__);
		return CMD_SEND_AND_RECIVE_CHECK_ERR;
	}
	DBG_ISP_SPL("Synchronizing with jmc/bcb........[Ok]\n\r");
	bat_log_insert(__LINE__);
	
	/*
	 *Send Write RAM,(W 1073742336 4096<CR><LF>)
	 *Receive Response (W 40000200H 4096<CR><LF>0<CR><LF>)
	 */
	if(CHIPLPC1768 == chipflag)
	{
		/*Write RAM W 10000200H 4096*/
		strcpy(sent_str, "W 268435968 4096\r\n");
		strcpy(ask_str,  "W 268435968 4096\r0\r\n");
	}
	else
	{
		/*Write RAM W 40000200H 4096*/
		strcpy(sent_str, "W 1073742336 4096\r\n");
		strcpy(ask_str,  "W 1073742336 4096\r\n0\r\n");
	}
	
	if(0 != cmd_communication(uart_id, sent_str, ask_str))
	{
		bat_log_insert(__LINE__);
		return CMD_SEND_AND_RECIVE_CHECK_ERR;
	}
	bat_log_insert(__LINE__);
	
	in_pos = 0;
	while(in_pos < data_len)
	{
	    /*No coding bytes reminder*/
		if(data_len - in_pos > MAX_LINELEN)
		{
			line_len = MAX_LINELEN;
		}	
		else
		{
			line_len = data_len - in_pos;
		}
		
		/*
		 *Finished 4095 bytes to send , one byte additionally, up to 4K .
		 */
		if((in_pos % 4096) == 4095)
		{
			/*To encode the additional one byte*/
		    out_buf[out_pos++] = ENCODE_BYTE(1);
			out_buf [out_pos++] = ENCODE_BYTE((buf_ptr[in_pos] & 0xFC) >> 2);
			out_buf [out_pos++] = ENCODE_BYTE((buf_ptr[in_pos] & 0x03) << 4);
			out_buf [out_pos++] = ENCODE_BYTE(0);
			out_buf [out_pos++] = ENCODE_BYTE(0);
			/*CRC*/
			crc += buf_ptr[in_pos];
			in_pos++;
			/*Write end indicator*/
			out_buf[out_pos++] = '\r';
			out_buf[out_pos++] = '\n';
			out_buf[out_pos++] = '\0';

			/*Send encoded data though UART*/
			str_len = strlen(out_buf);
			if(0 != send_data_to_flash(uart_id, out_buf, str_len))
			{
				bat_log_insert(__LINE__);
				return SEND_DATA_TO_FLASH_ERR2;
			}
			
			/*Send CRC*/
			sprintf(sent_str, "%d\r\n", crc);
			if(CHIPLPC1768 == chipflag)
			{
				sprintf(ask_str, "%d\rOK\r\n", crc);
			}
			else
			{
				sprintf(ask_str, "%d\r\nOK\r\n", crc);
			}

			if(0 != cmd_communication(uart_id, sent_str, ask_str))
			{
				bat_log_insert(__LINE__);
				return CMD_SEND_AND_RECIVE_CHECK_ERR;
			}
			crc = 0;
			out_pos = 0;
			
			if(CHIPLPC2132 == chipflag)
			{
				/*RAM to Flash*/
				if(sect >= 8)/* > 32k*/
				{
				    sprintf(sent_str, "P %d %d\r\n", 8, 8);
				}  
				else
				{
				    sprintf(sent_str, "P %d %d\r\n", sect, sect);
				}
			}
			else if(CHIPLPC2138 == chipflag)
			{
				sect_tmp = sect;
				if((sect_tmp > 7) && (sect_tmp < 125))
				{
					if(sect_tmp < 120)
					{
						sect_tmp = sect_tmp/8 + 7;
					}
					else
					{
						sect_tmp = sect_tmp - 120 + 22;
					}
				} 
				sprintf(sent_str, "P %d %d\r\n", sect_tmp, sect_tmp);
			}
			else if(CHIPLPC1768 == chipflag)
			{
				sect_tmp = sect;
				if((sect_tmp > 15) && (sect_tmp < 126))
				{
					if(sect_tmp < 120)
					{
						sect_tmp = sect_tmp/8 + 14;
					}
					else
					{
						sect_tmp = sect_tmp - 120 + 29;
					}
				} 				
				sprintf(sent_str, "P %d %d\r\n", sect_tmp, sect_tmp);
			}

			/*Prepare the sector to write  P X X*/
			strcpy(ask_str, sent_str);
			if(CHIPLPC1768 == chipflag)
			{
				ask_str[strlen(ask_str) - 1] = 0;
			}
			
			strcat(ask_str, "0\r\n");
			if(0 != cmd_communication(uart_id, sent_str, ask_str))
			{
				bat_log_insert(__LINE__);
				return CMD_SEND_AND_RECIVE_CHECK_ERR;
			}

			/*Copy RAM to Flash	C Y 1073742336 4096*/
			if(CHIPLPC1768 == chipflag)
			{
				sprintf(sent_str, "C %d 268435968 4096\r\n", sect*4096);
				strcpy(ask_str, sent_str);
				ask_str[strlen(ask_str) - 1] = 0;
			}
			else
			{
				sprintf(sent_str, "C %d 1073742336 4096\r\n", sect*4096);
				strcpy(ask_str, sent_str);
			}
			
			strcat(ask_str, "0\r\n");
			if(0 != cmd_communication(uart_id, sent_str, ask_str))
			{
				bat_log_insert(__LINE__);
				return CMD_SEND_AND_RECIVE_CHECK_ERR;
			}
			
			/*To the next flash sector*/
			sect++;           
			if(data_len - in_pos > 4096)     
			{	
				/*The remainder data grater than 4k£¬need prepare 4k RAM for use*/	
				/*"W 1073742336 4096\r\n"*/
				if(CHIPLPC1768 == chipflag)
				{
					strcpy(sent_str, "W 268435968 4096\r\n");
					strcpy(ask_str, sent_str);
					ask_str[strlen(ask_str) - 1] = 0;
				}
				else
				{
					strcpy(sent_str, "W 1073742336 4096\r\n");
					strcpy(ask_str, sent_str);
				}
				
				strcat(ask_str, "0\r\n");
				if(0 != cmd_communication(uart_id, sent_str, ask_str))
				{
					bat_log_insert(__LINE__);
					return CMD_SEND_AND_RECIVE_CHECK_ERR;
				}
			}
			else
			{
				/*"W 1073742336 X\r\n"*/
				if(CHIPLPC1768 == chipflag)
				{
					sprintf(sent_str, "W 268435968 %ld\r\n", data_len - in_pos);
					strcpy(ask_str, sent_str);
					ask_str[strlen(ask_str) - 1] = 0;
				}
				else
				{
					sprintf(sent_str, "W 1073742336 %ld\r\n", data_len - in_pos);
					strcpy(ask_str, sent_str);
				}
				
				strcat(ask_str, "0\r\n");
				if(0 != cmd_communication(uart_id, sent_str, ask_str))
				{
					bat_log_insert(__LINE__);
					return CMD_SEND_AND_RECIVE_CHECK_ERR;
				}
			}
		}
		else
		{
            /*Generate UU Codine line*/
			out_buf[out_pos++] = ENCODE_BYTE(line_len);
			/*UU encode*/
			for (line_cnt = line_len; line_cnt > 0; line_cnt -= 3)
			{
				/*bin to uu encode, 3 bytes to 4 bytes*/
				out_buf[out_pos++] = ENCODE_BYTE((buf_ptr[in_pos] & 0xFC) >> 2);
				out_buf[out_pos++] = ENCODE_BYTE(((buf_ptr[in_pos] & 0x03) << 4) +
										((buf_ptr[in_pos+1] & 0xF0) >> 4));
				out_buf[out_pos++] = ENCODE_BYTE(((buf_ptr[in_pos+1] & 0x0F) << 2) +
										((buf_ptr[in_pos+2] & 0xC0) >> 6));
				out_buf[out_pos++] = ENCODE_BYTE(buf_ptr[in_pos+2] & 0x3F);

				/*CRC*/
				crc += (buf_ptr[in_pos] + buf_ptr[in_pos + 1] + buf_ptr[in_pos + 2]);
				in_pos += 3;
			}

			/*Fix CRC*/
			if((line_len % 3) != 0)
			{
				crc -= (0xFF * (3 - (line_len % 3)));
			}
			
			/*Write end indicator*/
			out_buf[out_pos++] = '\r';
			out_buf[out_pos++] = '\n';
			out_buf[out_pos++] = '\0';
			
			/*Send encoded data though UART*/
			str_len = strlen(out_buf);
			if(0 != send_data_to_flash(uart_id, out_buf, str_len))
			{
				DBG_ISP_DTL("Send data to flash failed\n");
				bat_log_insert(__LINE__);
				return SEND_DATA_TO_FLASH_ERR1;
			}
			
			/*Finished this times*/
			out_pos = 0;
			
			/*Send CRC after 20 "UU encode line", each "UU encode line" contain 45 bytes*/
			if(((in_pos % 4096) % 900) == 0)
			{ 
				data_count++;
				tmp = (data_count * 900 * 100)/(float)data_len;
				DBG_ISP_DTL("Programming flash.................[%d%%]\r", (UINT32)tmp);
				/*Send CRC*/
				sprintf(sent_str, "%d\r\n", crc);
				if(CHIPLPC1768 == chipflag)
				{
					sprintf(ask_str, "%d\rOK\r\n", crc);
				}
				else
				{
					sprintf(ask_str, "%d\r\nOK\r\n", crc);
				}

				if(0 != cmd_communication(uart_id, sent_str, ask_str))
				{
					bat_log_insert(__LINE__);
					return CMD_SEND_AND_RECIVE_CHECK_ERR;
				}
				
				crc = 0;
			}
		}
	}

	if(0 != ((in_pos % 4096) % 900))
	{
		sprintf(sent_str, "%d\r\n", crc);
		if(CHIPLPC1768 == chipflag)
		{
			sprintf(ask_str, "%d\rOK\r\n", crc);
		}
		else
		{
			sprintf(ask_str, "%d\r\nOK\r\n", crc);
		}
		
		if(0 != cmd_communication(uart_id, sent_str, ask_str))
		{
			bat_log_insert(__LINE__);
			return CMD_SEND_AND_RECIVE_CHECK_ERR;
		}
	}

	if(CHIPLPC2132 == chipflag)
	{
		/*Prepare the sector to write  P X X*/
		if(sect >= 8)/* > 32k*/
		{
		    sprintf(sent_str, "P %d %d\r\n", 8, 8);
		}  
		else
		{
		    sprintf(sent_str, "P %d %d\r\n", sect, sect);
		}
	}
	else if(CHIPLPC2138 == chipflag)
	{
		sect_tmp = sect;
		/*4k bytes finished, need RAM->Flash once*/
		if((sect_tmp > 7) && (sect_tmp < 125))
		{
			if(sect_tmp < 120)
			{
				sect_tmp = sect_tmp/8 + 7;
			}
			else
			{
				sect_tmp = sect_tmp - 120 + 22;
			}
		} 
		sprintf(sent_str, "P %d %d\r\n", sect_tmp, sect_tmp);
	}
	else if(CHIPLPC1768 == chipflag)
	{
		sect_tmp = sect;
		if((sect_tmp > 15) && (sect_tmp < 126))
		{
			if(sect_tmp < 120)
			{
				sect_tmp = sect_tmp/8 + 14;
			}
			else
			{
				sect_tmp = sect_tmp - 120 + 29;
			}
		} 
		sprintf(sent_str, "P %d %d\r\n", sect_tmp, sect_tmp);
	}
	
	strcpy(ask_str, sent_str);
	if(CHIPLPC1768 == chipflag)
	{
		ask_str[strlen(ask_str) - 1] = 0;
	}
	strcat(ask_str, "0\r\n");
	
	if(0 != cmd_communication(uart_id, sent_str, ask_str))
	{
		bat_log_insert(__LINE__);
		return CMD_SEND_AND_RECIVE_CHECK_ERR;
	}
	
	/*Copy RAM to Flash C Y 1073742336 Z*/
	if(CHIPLPC1768 == chipflag)
	{
		sprintf(sent_str, "C %d 268435968 4096\r\n", sect*4096);
		strcpy(ask_str, sent_str);
		ask_str[strlen(ask_str) - 1] = 0;
	}
	else
	{
		sprintf(sent_str, "C %d 1073742336 4096\r\n", sect*4096);
		strcpy(ask_str, sent_str);
	}

	strcat(ask_str, "0\r\n");
	if(0 != cmd_communication(uart_id, sent_str, ask_str))
	{
		bat_log_insert(__LINE__);
		return CMD_SEND_AND_RECIVE_CHECK_ERR;
	}
	bat_log_insert(__LINE__);

	tmp = 100;
	DBG_ISP_DTL("Programming flash.................[%d%%]\r", (UINT32)tmp);
	bat_log_insert(__LINE__);

	return BMC_UPDATE_SUCCESS;
}

PRIVATE void ac_down_isr(UINT8 int_num)
{
	UINT8 log_phy_id = 0;
	UINT16 ac_down = 0;
	PHYMAP_TYPE bcast_map = {0};

	/* disable IntL interrupt for AC Down temporarily */
	cicint_int_disable(CICINT_INT_INTIB0);
	hal_time_busy_wait_us(20 * 1000);
	active_event_insert(SES_LOG_MODULE_INT, SES_LOG_EVENT_INT_AC_DOWN, 0, 0);
	
	for(log_phy_id = 0; log_phy_id < istr_phy_count_get(); log_phy_id++)
	{
		PHY_MAP_BIT_SET(bcast_map, log_phy_id);	
	}
	
	ecmr_bcast_map_tx(bcast_map, FALSE, ECMR_BROADCAST_EXPANDER);

	/* Enable this interrupt */
	cicint_int_enable(CICINT_INT_INTIB0);

	/*Really AC Down*/
	ac_down = CPLD_READ(CPLD_BASE_ADDR, CPLD_REG_POWER_STATUS);
	if((ac_down & 0x03) == 0)
	{
		if((odsp_resvered_disks_map != 0) || bat_polling)
		{
			disk_poweroff_soon(odsp_resvered_disks_map);
			bat_log_insert(0x5A5A);
			bat_log_insert((UINT16)(odsp_resvered_disks_map & 0xFFFF));
			bat_log_insert((UINT16)((odsp_resvered_disks_map >> 16) & 0xFFFF));
		}
		else
		{
			/*No resvered disks , just poweroff itself*/
			CPLD_POWER_OFF_LOCAL;
		}
	}
	else
	{
		/*Log it*/
		bat_log_insert(0xA55A);
		bat_log_insert(ac_down);
		bat_log_insert((UINT16)bat_polling);
	}
	
	return;
}

PUBLIC battery_bmc_isp_task_parms_struct* battery_bmc_isp_task_parms_get( osf_sys_cfg_struct  *osf_cfg_ptr )
{
    /* Allocate storage for parameter blocks. */
    battery_bmc_isp_task_parms_ptr = calloc(sizeof(battery_bmc_isp_task_parms_struct), 1);

    /* 
    ** Check that malloc passes. 
    ** By design there should be no errors with memory allocation at this point 
    */
    /**************************error string should rewrite***************************/
    PMCFW_ASSERT(NULL != battery_bmc_isp_task_parms_ptr, PMCFW_ERR_INVALID_PARAMETERS);
	/**************************error string should rewrite***************************/
	
    /* Default values       */
    battery_bmc_isp_task_parms_ptr->config.thread_stack_size = BATTERY_BMC_ISP_TASK_STACK_SIZE_DEFAULT;      
    battery_bmc_isp_task_parms_ptr->config.thread_priority   = BATTERY_BMC_ISP_TASK_PRIORITY_DEFAULT; 
    battery_bmc_isp_task_parms_ptr->battery_bmc_isp_task_period = BATTERY_BMC_ISP_TASK_PERIOD_DEFAULT;   

	/* Get OSF resources */
    osf_cfg_ptr->num_threads   += BATTERY_BMC_ISP_TASK_NUM_THREADS;
    osf_cfg_ptr->num_mbx       += BATTERY_BMC_ISP_TASK_NUM_MBX;
    osf_cfg_ptr->num_mem_pools += BATTERY_BMC_ISP_TASK_NUM_POOLS; 
    
    return battery_bmc_isp_task_parms_ptr;
    
}

PRIVATE void battery_bmc_isp_uart_isr(UINT8 int_num)
{    
    uart_isr_decode(int_num - CICINT_INT_UART0);
}

PUBLIC void battery_bmc_isp_task_init(void)
{
	if(PRODUCT_IS_SSU2125 || PRODUCT_IS_SSU2225)
	{
		cicint_int_set(CICINT_INT_UART2, battery_bmc_isp_uart_isr);    
		cicint_int_enable(CICINT_INT_UART2);

		uart_init(BATTERY_UPGRADE_UART_PORT,
				(UINT32)UART2_BASEADDR,
		      	8,
		     	1,
		     	UART_PARITY_NONE,
		      	UART_BAUD_57600,
				FALSE);
	}

	if(PRODUCT_IS_SSU2125 || PRODUCT_IS_SSU2225)
	{	
		/* Register IntL interrupt for AC Down */
		cicint_int_set(CICINT_INT_INTIB0, ac_down_isr);
		cicint_int_enable(CICINT_INT_INTIB0);	
	}
}

PUBLIC void battery_bmc_isp_task_main(UINT32 input)
{	
	CHAR rx_ch = 0;
	UINT8 retry = 0, ret = 0;
	UINT16 bat_inslot = 0;
	UINT32 cached_start_addr = 0, length = 0;
	PMCFW_ERROR rv = PMC_SUCCESS;
	flm_hndl flm_handle;
	flm_partition_info_struct	flm_info;
    flm_partition_enum	dst_par = FLM_PARTITION_DATA1;

	osf_thr_struct *thr_ptr = NULL;
	thr_ptr = (osf_thr_struct *)osf_thread_hndl_self();
	thread_id_bmc_isp = thr_ptr->thread_id;
	thr_ptr = NULL;

	osf_thread_sleep(osf_time_ms_to_ticks(4000));

	while(TRUE)
	{
		if(battery_upgrade_is_approved)
		{
			/*if opp is updating bat, so we just waiting, this should never be happen
			 *in fact, odsp should make sure that just one SP is execting the upgrade 
			 *process, but do this for balabala...*/
			if(BAT_IS_UPGRADING(opper_bat_upgrade_status))
			{
				bat_log_insert(__LINE__);
				goto end;
			}
			
			if(! get_bat_access_authority())
			{
				bat_log_insert(__LINE__);
				goto end;
			}

			bat_log_clean();

			/*Sleep 20s because data1 partation will be upgrade twice, and after 
			 *the first time finished , the tag of "battery_upgrade_is_approved"
			 *have been set to "true", so that time the data1 is not ready to 
			 *be written to BAT, we just need wait,wait,wait...
			 *													lyy # 2014.12.22
			 */
			osf_thread_sleep(osf_time_s_to_ticks(20));
			
			rv = flm_init(&flm_handle);
		    if (PMC_SUCCESS != rv)
		    {
		        goto end;
		    }
			
		    flm_partition_info_get(flm_handle, dst_par, &flm_info);
			cached_start_addr = flm_info.start_addr & 0x1FFFFFFF;    
   			cached_start_addr |= 0x80000000;
			DBG_ISP_DTL("\ncached_start_addr = %#x\n", cached_start_addr);

			length = (UINT32)(*((UINT32 *)(flm_info.end_addr - PARTITION_FW_LENGTH_NEGATIVE_OFFSET)));
			DBG_ISP_DTL("length = %#x\n", length);
			if(length == 0xFFFFFFFF)
			{
				/*No valid data is proved*/
				goto end;
			}

			/*Beging to upgrade "battery one", the left battery when you look from disk side*/
			bat_to_upgrade = BAT_NUMBER_BAT_A;
			bat_inslot = CPLD_READ(CPLD_BASE_ADDR, CPLD_REG_BATTERY_INSLOT);
			if(bat_inslot & CPLD_MASK_BAT_LEFT_INSLOT)
			{
				/* bat 1 is not inslot */
				DBG_ISP_DTL("Battery 1 is not inslot\n");
				goto bat2;
			}

			bat_log_insert(__LINE__);
			local_bat_upgrade_status = BAT_UPGRADE_STATUS_UPDATING_BAT1;
			osf_thread_sleep(osf_time_ms_to_ticks(500));
			
			CPLD_WRITE(CPLD_BASE_ADDR, CPLD_REG_SXP_UART_SEL_CHECK, 0xC);	/*"M"*/
			osf_thread_sleep(osf_time_ms_to_ticks(2));
			CPLD_WRITE(CPLD_BASE_ADDR, CPLD_REG_SXP_UART_SEL, 0x08);		/*Reset Local BMC*/
			osf_thread_sleep(osf_time_ms_to_ticks(2));
			CPLD_WRITE(CPLD_BASE_ADDR, CPLD_REG_SXP_UART_SEL_CHECK, 0xC);	/*"M"*/
			osf_thread_sleep(osf_time_ms_to_ticks(2));
			CPLD_WRITE(CPLD_BASE_ADDR, CPLD_REG_SXP_UART_SEL, 0x06);		/*Change UART to Local BMC*/
			osf_thread_sleep(osf_time_ms_to_ticks(200));

			retry = 0;
			do
			{
				/*Clean the uart rx buffer before retry		lyy # 2016.02.22*/
				while(PMC_SUCCESS == uart_rx(BATTERY_UPGRADE_UART_PORT, &rx_ch));
				
				retry++;
				ret = program_flash(BATTERY_UPGRADE_UART_PORT, (UCHAR *)cached_start_addr, length);
				
				if(BMC_UPDATE_SUCCESS != ret)
				{
					bat_log_insert(__LINE__);
					local_bat_upgrade_status |= BAT_UPGRADE_STATUS_UPGRADE_FAILED_BAT1;
					/*Upgrade failed, Do sth to recovery*/
					CPLD_WRITE(CPLD_BASE_ADDR, CPLD_REG_SXP_UART_SEL_CHECK, 0xC);	/*"M"*/
					osf_thread_sleep(osf_time_ms_to_ticks(2));
					CPLD_WRITE(CPLD_BASE_ADDR, CPLD_REG_SXP_UART_SEL, 0x08);		/*Reset Local BMC*/
					osf_thread_sleep(osf_time_ms_to_ticks(2));
					CPLD_WRITE(CPLD_BASE_ADDR, CPLD_REG_SXP_UART_SEL_CHECK, 0xC);	/*"M"*/
					osf_thread_sleep(osf_time_ms_to_ticks(2));
					CPLD_WRITE(CPLD_BASE_ADDR, CPLD_REG_SXP_UART_SEL, 0x06);		/*Change UART to Local BMC*/

					if(retry < 3)
					{
						DBG_ISP_DTL("\nBAT1 Update failed ,waitting 2s for retry...\n");
						osf_thread_sleep(osf_time_ms_to_ticks(2000));
					}					
				}
			}while((BMC_UPDATE_SUCCESS != ret) && (retry < 3));		

			bat_log_insert(__LINE__);
			CPLD_WRITE(CPLD_BASE_ADDR, CPLD_REG_SXP_UART_SEL_CHECK, 0xC);	/*"M"*/
			osf_thread_sleep(osf_time_ms_to_ticks(2));
			CPLD_WRITE(CPLD_BASE_ADDR, CPLD_REG_SXP_UART_SEL, 0x08);		/*Reset Local BMC*/
			osf_thread_sleep(osf_time_ms_to_ticks(2));
			CPLD_WRITE(CPLD_BASE_ADDR, CPLD_REG_SXP_UART_SEL_CHECK, 0xC);	/*"M"*/
			osf_thread_sleep(osf_time_ms_to_ticks(200));
			CPLD_WRITE(CPLD_BASE_ADDR, CPLD_REG_SXP_UART_SEL, 0x00);

			if((BMC_UPDATE_SUCCESS != ret) && (3 == retry))
			{
				local_bat_upgrade_status &= ~BAT_UPGRADE_STATUS_UPDATING_BAT1;
				DBG_ISP_DTL("BAT1 update failed.\n");

				/*Clean the uart rx buffer before retry		lyy # 2016.02.22*/
				while(PMC_SUCCESS == uart_rx(BATTERY_UPGRADE_UART_PORT, &rx_ch));
			}
			else
			{
				local_bat_upgrade_status = BAT_UPGRADE_STATUS_UPGRADE_SUCCESS_BAT1;
				DBG_ISP_DTL("BAT1 update success.\n");
			}

		bat2:			
			/*Beging to upgrade "battery two", the right battery when you look from disk side*/
			bat_to_upgrade = BAT_NUMBER_BAT_B;
			bat_inslot = CPLD_READ(CPLD_BASE_ADDR, CPLD_REG_BATTERY_INSLOT);
			if(bat_inslot & CPLD_MASK_BAT_RIGHT_INSLOT)
			{
				/* bat 1 is not inslot */
				DBG_ISP_DTL("Battery 2 is not inslot\n");
				goto over;
			}
			
			local_bat_upgrade_status |= BAT_UPGRADE_STATUS_UPDATING_BAT2;
			CPLD_WRITE(CPLD_BASE_ADDR, CPLD_REG_SXP_UART_SEL_CHECK, 0xC);	/*"M"*/
			osf_thread_sleep(osf_time_ms_to_ticks(2));
			CPLD_WRITE(CPLD_BASE_ADDR, CPLD_REG_SXP_UART_SEL, 0x09);		/*Reset Oppo BMC*/
			osf_thread_sleep(osf_time_ms_to_ticks(2));
			CPLD_WRITE(CPLD_BASE_ADDR, CPLD_REG_SXP_UART_SEL_CHECK, 0xC);	/*"M"*/
			osf_thread_sleep(osf_time_ms_to_ticks(2));
			CPLD_WRITE(CPLD_BASE_ADDR, CPLD_REG_SXP_UART_SEL, 0x04);		/*Change UART to Oppo BMC*/
			osf_thread_sleep(osf_time_ms_to_ticks(200));
			
			retry = 0;
			do
			{
				/*Clean the uart rx buffer before retry		lyy # 2016.02.22*/
				while(PMC_SUCCESS == uart_rx(BATTERY_UPGRADE_UART_PORT, &rx_ch));
				
				retry++;
				ret = program_flash(BATTERY_UPGRADE_UART_PORT, (UCHAR *)cached_start_addr, length);
				
				if(BMC_UPDATE_SUCCESS != ret)
				{
					bat_log_insert(__LINE__);
					local_bat_upgrade_status |= BAT_UPGRADE_STATUS_UPGRADE_FAILED_BAT2;
					/*Upgrade failed, Do sth to recovery*/
					CPLD_WRITE(CPLD_BASE_ADDR, CPLD_REG_SXP_UART_SEL_CHECK, 0xC);	/*"M"*/
					osf_thread_sleep(osf_time_ms_to_ticks(2));
					CPLD_WRITE(CPLD_BASE_ADDR, CPLD_REG_SXP_UART_SEL, 0x09);		/*Reset Oppo BMC*/
					osf_thread_sleep(osf_time_ms_to_ticks(2));
					CPLD_WRITE(CPLD_BASE_ADDR, CPLD_REG_SXP_UART_SEL_CHECK, 0xC);	/*"M"*/
					osf_thread_sleep(osf_time_ms_to_ticks(2));
					CPLD_WRITE(CPLD_BASE_ADDR, CPLD_REG_SXP_UART_SEL, 0x04);		/*Change UART to Oppo BMC*/

					if(retry < 3)
					{
						DBG_ISP_DTL("\nBAT2 Update failed ,waitting 2s for retry...\n");
						osf_thread_sleep(osf_time_ms_to_ticks(2000));
					}
				}
			}while((BMC_UPDATE_SUCCESS != ret) && (retry < 3));
			
			bat_log_insert(__LINE__);
			CPLD_WRITE(CPLD_BASE_ADDR, CPLD_REG_SXP_UART_SEL_CHECK, 0xC);	/*"M"*/
			osf_thread_sleep(osf_time_ms_to_ticks(2));
			CPLD_WRITE(CPLD_BASE_ADDR, CPLD_REG_SXP_UART_SEL, 0x09);		/*Reset Oppo BMC*/
			osf_thread_sleep(osf_time_ms_to_ticks(2));
			CPLD_WRITE(CPLD_BASE_ADDR, CPLD_REG_SXP_UART_SEL_CHECK, 0xC);	/*"M"*/
			osf_thread_sleep(osf_time_ms_to_ticks(2));
			CPLD_WRITE(CPLD_BASE_ADDR, CPLD_REG_SXP_UART_SEL, 0x00);
			if((BMC_UPDATE_SUCCESS != ret) && (3 == retry))
			{
				local_bat_upgrade_status &= ~BAT_UPGRADE_STATUS_UPDATING_BAT2;
				DBG_ISP_DTL("BAT2 update failed.\n");

				/*Clean the uart rx buffer before retry		lyy # 2016.02.22*/
				while(PMC_SUCCESS == uart_rx(BATTERY_UPGRADE_UART_PORT, &rx_ch));
			}
			else
			{
				local_bat_upgrade_status &= ~BAT_UPGRADE_STATUS_UPDATING_BAT2;
				local_bat_upgrade_status |= BAT_UPGRADE_STATUS_UPGRADE_SUCCESS_BAT2;
				DBG_ISP_DTL("BAT2 update success.\n");
			}

		over:
			/*All over*/
			battery_upgrade_is_approved = FALSE;
			flm_partition_erase(flm_handle, dst_par);
		}
	end:
		osf_thread_sleep(osf_time_ms_to_ticks(battery_bmc_isp_task_parms_ptr->battery_bmc_isp_task_period));
	}
}

PUBLIC void battery_bmc_isp_task_create(battery_bmc_isp_task_parms_struct *parms_ptr)
{
	PMCFW_ASSERT(parms_ptr == battery_bmc_isp_task_parms_ptr, PMCFW_ERR_INVALID_PARAMETERS);
	
    /* Create thread */
    (void)osf_thread_create("BAT BMC ISP Task",				/* name       */
                      &battery_bmc_isp_task_main,          	/* Thread fn  */
                      (UINT32) parms_ptr,                      /* Input      */
                      parms_ptr->config.thread_stack_size,     /* Stack size */
                      parms_ptr->config.thread_priority );     /* Priority   */
}

PUBLIC void* battery_bmc_isp_log_get(UINT8 *num)
{
	*num = ((BATTERY_UPGRADE_LOG_MAX_NUM > UINT8_MAX) ? UINT8_MAX : BATTERY_UPGRADE_LOG_MAX_NUM);
	return (void *)bat_upgrade_log;
}

