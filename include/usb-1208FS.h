/*
 *
 *  Copyright (c) 2004-2005  Warren Jasper <wjasper@tx.ncsu.edu>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */


#ifndef USB_1208FS_H
#define USB_1208FS_H

#ifdef __cplusplus
extern "C" {
#endif

#define USB1208FS_PID (0x0082)

#define DIO_PORTA (0x00)
#define DIO_PORTB (0x01)

#define DIO_DIR_IN  (0x01)
#define DIO_DIR_OUT (0x00)

/* Commands and HID Report ID for USB 1208FS  */
#define DCONFIG     (0x01)     
#define DIN         (0x03)     
#define DOUT        (0x04)     

#define AIN         (0x10)     
#define AIN_SCAN    (0x11)     
#define AIN_STOP    (0x12)     
#define ALOAD_QUEUE (0x13)     

#define AOUT        (0x14)     
#define AOUT_SCAN   (0x15)     
#define AOUT_STOP   (0x16)     

#define CINIT       (0x20)     
#define CIN         (0x21)     

#define MEM_READ    (0x30)     
#define MEM_WRITE   (0x31)     

#define BLINK_LED   (0x40)     
#define RESET       (0x41)     
#define SET_TRIGGER (0x42)     
#define SET_SYNC    (0x43)     
#define GET_STATUS  (0x44)     
#define SET_CAL     (0x45)     
#define GET_ALL     (0x46)     

#define PREPARE_DOWNLOAD (0x50)
#define WRITE_CODE       (0x51)
#define WRITE_SERIAL     (0x53)
#define READ_CODE        (0x55)






#define EXT_TRIG_FAILING_EDGE 0;
#define EXT_TRIG_RAISING_EDGE 1;
#define SYNC_MASTER 0
#define SYNC_SLAVE  1

#define SE_10_00V  (0x9)           // Single Ended 0-10.0 V

#define BP_20_00V  (0x0)           // Differential +/- 20.0 V
#define BP_10_00V  (0x1)           // Differential +/- 10.0 V
#define BP_5_00V   (0x2)           // Differential +/- 5.00 V
#define BP_4_00V   (0x3)           // Differential +/- 4.00 V
#define BP_2_50V   (0x4)           // Differential +/- 2.50 V
#define BP_2_00V   (0x5)           // Differential +/- 2.00 V
#define BP_1_25V   (0x6)           // Differential +/- 1.25 V
#define BP_1_00V   (0x7)           // Differential +/- 1.00 V

#define AIN_EXECUTION     0x1  
#define AIN_TRANSFER_MODE 0x2  
#define AIN_TRIGGER       0x4  
#define AIN_DEBUG         0x8  
#define AIN_GAIN_QUEUE    0x10 

/* function prototypes for the USB-1208FS */
void usbDConfigPort_USB1208FS(HIDInterface* hid, __u8 port, __u8 direction);
void usbDIn_USB1208FS(HIDInterface* hid, __u8 port, __u8* din_value);
void usbDOut_USB1208FS(HIDInterface* hid, __u8 port, __u8 value);

signed short usbAIn_USB1208FS(HIDInterface* hid, __u8 channel, __u8 range);
void usbAOut_USB1208FS(HIDInterface* hid, __u8 channel, __u16 value);
int usbAOutScan_USB1208FS(HIDInterface* hid[], __u8 lowchannel, __u8 highchannel,
			  __u32 count, float *frequency, __u16 data[], __u8 options);
void usbAOutStop_USB1208FS(HIDInterface* hid);
void usbAInStop_USB1208FS(HIDInterface* hid);
int usbAInScan_USB1208FS(HIDInterface* hid[], __u8 lowchannel, __u8 highchannel, __u32 count,
			 float *frequency, __u8 options, __s16 data[]);
int usbAInScan_USB1208FS_SE(HIDInterface* hid[], __u8 lowchannel, __u8 highchannel, __u32 count,
			 float *frequency, __u8 options, __s16 data[]);
void usbALoadQueue_USB1208FS(HIDInterface* hid, __u8 num, __u8 chan[], __u8 gains[]);

void usbInitCounter_USB1208FS(HIDInterface* hid);
__u32 usbReadCounter_USB1208FS(HIDInterface* hid);

void usbReadMemory_USB1208FS( HIDInterface* hid, __u16 address, __u8 count, __u8 memory[]);
int usbWriteMemory_USB1208FS(HIDInterface* hid, __u16 address,  __u8 count, __u8 data[]);
void usbBlink_USB1208FS(HIDInterface* hid);
int usbReset_USB1208FS(HIDInterface* hid);
__u16 usbGetStatus_USB1208FS(HIDInterface* hid);
void usbSetTrigger_USB1208FS(HIDInterface* hid, __u8 type);
void usbSetSync_USB1208FS(HIDInterface* hid, __u8 type);
void usbGetAll_USB1208FS(HIDInterface* hid, __u8 data[]);
float volts_FS(const int gain, const signed short num);
float volts_SE(const signed short num);

#ifdef __cplusplus
} /* closing brace for extern "C" */
#endif

#endif 
