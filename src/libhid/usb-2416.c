/*
 *  Copyright (c) 2009  Warren Jasper <wjasper@tx.ncsu.edu>
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

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <asm/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>

#include "usb-2416.h"

#define HS_DELAY 1000

void usbBuildGainTable_USB2416(usb_dev_handle *udev, double table[NGAINS][2])
{
  /*
    Builds a lookup table of calibration coefficents to translate values into voltages:
         voltage = value*table[gain#][0] + table[gain#][1]
    only needed for fast lookup.
  */
  int j, k;
  __u16 address = 0x00A0;

  for (j = 0; j < NGAINS; j++) {
    for (k = 0; k < 2; k++) {
      usbReadMemory_USB2416(udev, 8, address, (__u8 *) &table[j][k]);
      address += 0x8;
    }
  }
  return;
}

void usbBuildGainTable_USB2416_4AO(usb_dev_handle *udev, double table_AO[NCHAN_AO][2])
{
  /*
    Builds a lookup table of calibration coefficents to translate values into voltages:
       corrected value = value*table[VDAC#][0] + table[VDAC][1]
  */

  int j, k;
  __u16 address = 0x0180;

  for (j = 0; j < NCHAN_AO; j++) {
    for (k = 0; k < 2; k++) {
      usbReadMemory_USB2416(udev, 8, address, (__u8 *) &table_AO[j][k]);
      address += 0x8;
    }
  }
  return;
}

/***********************************************
 *            Digital I/O                      *
 ***********************************************/

/* reads digital port  */
__u8  usbDIn_USB2416(usb_dev_handle *udev, __u8 port)
{
  /*
    This command reads the current state of the DIn port.
    port:  0  onboard (pins 0-7)
           1  Expansion 1 (pins 8-15)
           2  Expansion 2 (pins 16-23)
  */
  __u8 requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  __u8 data = 0x0;

  usb_control_msg(udev, requesttype, DIN, (__u16) port,  0x0, (char *) &data, sizeof(data), HS_DELAY);
  return data;
}

/* read/writes digital port latch */
void usbDOut_USB2416(usb_dev_handle *udev, __u8 value, __u8 port)
{
  /*
    This command writes the DOut port latch.
    port:  0  onboard (pins 0-7)
           1  Expansion 1 (pins 8-15)
           2  Expansion 2 (pins 16-23)

	   NOTE: The DIO are open-drain, which when used as an output is capable of sinking up to 150 mA.
	   Writing a "1" to a bit will cause its voltage to go LOW (0V), and writing a "0" to
	   the bit will cause the voltage to go HIGH (5V) by the 47k Ohm pullup resister.
	   See page 23 of the users manual.
  */
  __u8 requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  char buf[2];

  buf[0] = port;
  buf[1] = value;

  usb_control_msg( udev, requesttype, DOUT, 0x0, 0x0, buf, sizeof(buf), HS_DELAY );
  return;
}

__u8 usbDOutR_USB2416(usb_dev_handle *udev, __u8 port)
{
  /*
    This command reads the DOut port latch.
  */
  __u8 requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  __u8 data;

  usb_control_msg(udev, requesttype, DOUT, port, 0x0, (char *) &data, sizeof(data), HS_DELAY);
  return data;
}

/***********************************************
 *            Analog Input                     *
 ***********************************************/

#define  SIGN_BITMASK (1 << 23)
#define  FULL_SCALE24_BITMASK ((1<<24) - 1)
#define  SIGN_EXT_BITMASK (~FULL_SCALE24_BITMASK)

int  sint24TOint(int int24val)
// Converts a 2's complement signed 24 bit number to a int (32 or 64 bit)
{
  if (int24val & SIGN_BITMASK) {
    int24val |= SIGN_EXT_BITMASK;
  } else {
    int24val &= FULL_SCALE24_BITMASK;
  }

  return int24val;
}

__u32  intTOsint24(int int32)
{
  if (int32 < 0) {
    int32 &= SIGN_EXT_BITMASK;
    int32 |= SIGN_BITMASK;
  } else {
    int32 &= FULL_SCALE24_BITMASK;
  }
  return int32;
}

int usbAIn_USB2416(usb_dev_handle *udev, __u8 channel, __u8 mode, __u8 range, __u8 rate, __u8 *flags)
{
  __u32 data;
  __u16 input1;
  __u16 input2;
  __u8 requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  input1 = (mode << 8) | channel;
  input2 = (rate << 8) | range;


  usb_control_msg(udev, requesttype, AIN, input1, input2, (char *) &data, sizeof(data), HS_DELAY);
  //  printf("input1 = %#x    input2 = %#x    data = %#x\n", input1, input2, data);
  *flags = (data >> 24);
  data &= 0x00ffffff;
  return sint24TOint(data);
}

void usbAInScanStop_USB2416(usb_dev_handle *udev)
{
  /*
    This command stops the analog input scan (if running)
  */
  
  __u8 requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  usb_control_msg(udev, requesttype, AIN_SCAN_STOP, 0x0, 0x0, NULL, 0x0, HS_DELAY);
}

__u8  usbAInScanStatus_USB2416(usb_dev_handle *udev, __u16 *depth)
{
  /*
    This command reads the status of the analog input scan.
  */
  __u8 requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  struct t_AInStatus {
    __u16 depth;    // number of samples currently in the FIFO (max 512)
    __u8 status;    // bit 0: 1 = scan running
                    // bit 1: 1 = scan overrun due to fifo full
                    // bit 2: 1 = scan overrun due to pacer period too short for queue
                    // bit 3-7: reserved
  } AInStatus;
  usb_control_msg(udev, requesttype, AIN_SCAN_STATUS, 0x0, 0x0, (char *) &AInStatus, sizeof(AInStatus), HS_DELAY);
  *depth =  AInStatus.depth;
  return AInStatus.status;
}

void usbAInScanQueueWrite_USB2416(usb_dev_handle *udev, AInScanQueue *queue)
{
  /*
    This command reads or writes the analog input scan channel queue.  The
    queue may have a maximum of 64 entries.  The queue can not be mondified
    during an AInScan.
  */
  
  __u8 requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (queue->count > MAX_QUEUE_SIZE) queue->count = MAX_QUEUE_SIZE;
  usb_control_msg(udev, requesttype, AIN_SCAN_QUEUE, 0x0, 0x0, (char *) queue, (1+queue->count*4), HS_DELAY);
}

void usbAInScanQueueRead_USB2416(usb_dev_handle *udev, AInScanQueue *queue)
{
  __u8 requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  usb_control_msg(udev, requesttype, AIN_SCAN_QUEUE, 0x0, 0x0, (char *) queue, sizeof(AInScanQueue), HS_DELAY);
}

double usbAInMinPacerPeriod_USB2416(usb_dev_handle *udev)
{
  AInScanQueue scanQueue;
  double period = 0.0;
  int i;

  // Calculate the minimum allowable pacer period
  usbAInScanQueueRead_USB2416(udev, &scanQueue);
  for(i = 0; i < scanQueue.count; i++) {
    switch(scanQueue.queue[i].rate) {
      case HZ30000: period += 1./30000. + 640.E-6; break;
      case HZ15000: period += 1./15000. + 640.E-6; break;
      case HZ7500:  period += 1./7500.  + 640.E-6; break;
      case HZ3750:  period += 1./3750.  + 640.E-6; break;
      case HZ2000:  period += 1./2000.  + 640.E-6; break;
      case HZ1000:  period += 1./1000.  + 640.E-6; break;
      case HZ500:   period += 1./500.   + 640.E-6; break;
      case HZ100:   period += 1./100.   + 640.E-6; break;
      case HZ60:    period += 1./60.    + 640.E-6; break;
      case HZ50:    period += 1./50.    + 640.E-6; break;
      case HZ30:    period += 1./30.    + 640.E-6; break;
      case HZ25:    period += 1./25.    + 640.E-6; break;
      case HZ15:    period += 1./15.    + 640.E-6; break;
      case HZ10:    period += 1./10.    + 640.E-6; break;
      case HZ5:     period += 1./5.     + 640.E-6; break;
      case HZ2_5:   period += 1./2.5    + 640.E-6; break;      
      default:  printf("Unknown rate.\n"); break;
    }      
  }
  return period;
}

void usbAInScanStart_USB2416(usb_dev_handle *udev, double frequency, __u16 count, __u8 packet_size, int *data)
{
  /*
    This command starts an analog input channel scan.  The channel
    configuration for the scan is set with AInScanQueue_US2416Write().
    This command will result in a bus stall if usbAInScan is currently
    running.
  */

  double period = 0.0;
  int nbytes;
  int ret = -1;
  __u32 pacer_period;
  __u16 depth;
  __u8 requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  __u8 status;

  struct t_scanPacket {
    __u8 pacer_period[4]; // pacer timer period = 50 kHz / (sample frequency)
    __u8 count[2];        // the total number of scans to perform (0 = continuous)
    __u8 packet_size;     // the number of samples per bulk transfer (0-15)
  } scanPacket;

  period = usbAInMinPacerPeriod_USB2416(udev);
  if (period > 1./frequency) {
    pacer_period = rint(period*50000.);
  } else {
    pacer_period = rint(50000./frequency);
  }
  
  memcpy(scanPacket.pacer_period, &pacer_period, 4);
  memcpy(scanPacket.count, &count, 2);
  scanPacket.packet_size = packet_size;

  if (usbAInScanStatus_USB2416(udev, &depth) & INPUT_SCAN_RUNNING) {
    printf("There are currently %d samples in the FIFO buffer.\n", depth);
    return;
  }
  usb_control_msg(udev, requesttype, AIN_SCAN_START, 0x0, 0x0, (char *) &scanPacket, sizeof(scanPacket), HS_DELAY);

  nbytes = (packet_size+1)*sizeof(int);
  status = usbStatus_USB2416(udev);

  while (status & INPUT_SCAN_RUNNING) {
    ret = usb_bulk_read(udev, USB_ENDPOINT_IN|1, (char *) data, nbytes, HS_DELAY);
    if (ret <= 0) {
      return;
    } else {
      data += ret/sizeof(int);
    }
    status = usbStatus_USB2416(udev);
  }
  
}
/***********************************************
 *          Analog Output                      *
 ***********************************************/
void usbAOutScanStop_USB2416_4AO(usb_dev_handle *udev)
{
  /* This command stops the analog output scan (if running) and
     clears the output FIFO data.  Any data in the endpoint buffers will
     be flushed, so this command is useful to issue prior to the
     beginning of an output scan.
  */

  __u8 requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  usb_control_msg(udev, requesttype, AOUT_SCAN_STOP, 0x0, 0x0, NULL, 0x0, HS_DELAY);
}

__u8  usbAOutScanStatus_USB2416_4AO(usb_dev_handle *udev, __u16 *depth)
{
  /*  This comamnd reads the status of the analog output scan:
      depth: the number of samples currently in the FIFO (max 1024)
      status: bit 0: 1 = scan running
              bit 1: 1 = scan underrun
              bits 2-7: reserved
  */
  __u8 requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  struct t_AOutStatus {
    __u16 depth;    // number of samples currently in the FIFO (max 512)
    __u8 status;    // bit 0: 1 = scan running
                    // bit 1: 1 = scan overrun due to fifo full
                    // bit 2: 1 = scan overrun due to pacer period too short for queue
                    // bit 3-7: reserved
  } AOutStatus;
  usb_control_msg(udev, requesttype, AOUT_SCAN_STATUS, 0x0, 0x0, (char *) &AOutStatus, sizeof(AOutStatus), HS_DELAY);
  *depth =  AOutStatus.depth;
  return AOutStatus.status;
}

void usbAOutScanStart_USB2416_4AO(usb_dev_handle *udev, double frequency, __u16 scans, __u8 options)
{
  /* This command configures the analog output channel scan.
     This command will result in a bus stall if an AOUT_SCAN is
     currently running.

     Notes:
     The output scan operates with the host continuously transferring data for the
     outputs until the end of the scan.  If the "scans" parameter is 0, the scan will run
     until the AOutScanStop command is issued by the host; if it is nonzero, the scan
     will stop automatically after the specified number of scans have been output.
     The channels in the scan are selected in the options bit field.  "Scans" refers to
     the number of updates to the channels (if all channels are used, one scan s an
     update to all 4 channels).

     period = 50kHz / frequency

     Multiple channels are updated simultaneously using the same time base.

     The output data is sent using the bulk out endpoint.  The data format is:
     low channel sample 0 : ... : [high channel sample 0]
     low channel sample 1 : ... : [high channel sample 1]
     .
     .
     .
     low channel sample n : ... : [high channel sample n]

     The output data is written to a 512-sample FIFO in the device.  The bulk endpoint
     data is only accepted if there is room in the FIFO.  Output data may be sent to the
     FIFO before the start of the scan, and the FIFO is cleared when the AOutScanStop command
     is received.  The scan will not begin until the command is sent (and output data is in
     the FIFO).  Data will be output until reaching the specified number of scans (in single
     execution mode)or an AOutScanStop command is sent.
  */
  __u8 requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  struct t_scanPacket {
    __u16 pacer_period;  // pacer timer period = 50 kHz / (scan frequency)
    __u8 scans[2];       // the total number of scans to perform (0 = continuous)
    __u8 options;        // bit 0: 1 = include channel 0 in output scan
			 // bit 1: 1 = include channel 1 in output scan
			 // bit 2: 1 = include channel 2 in output scan
			 // bit 3: 1 = include channel 3 in output scan
			 // bits 4-7 reserved
  } scanPacket;
  __u16 depth;
  
  scanPacket.pacer_period = (__u16) rint(50000./frequency);
  memcpy(scanPacket.scans, &scans, 2);
  scanPacket.options = options;

  if (usbAOutScanStatus_USB2416_4AO(udev, &depth) & OUTPUT_SCAN_RUNNING) {
    printf("There are currently %d samples in the Output FIFO buffer.\n", depth);
    return;
  }
  usb_control_msg(udev, requesttype, AOUT_SCAN_START, 0x0, 0x0, (char *) &scanPacket, sizeof(scanPacket), HS_DELAY);
}

void usbAOut_USB2416_4AO(usb_dev_handle *udev, int channel, double voltage, double table_AO[NCHAN_AO][2])
{
  /* This command writes the values for the analog output channels.  The
     values are 16-bit signed numbers.  This command will result in a control
     pipe stall if an output scan is running.  The equation for the output voltage is:

           V_out = (value / 2^15)* V_ref

     where "value" is the value written to the channel and V_ref = 10V.  
  */
  double dvalue;
  __u16 depth;
  short int value;
  __u8 requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  struct t_aOut {
    __u8 value[2];
    __u8 command;
  } aOut;

  dvalue = voltage*(1<<15)/10.;
  dvalue = dvalue*table_AO[channel][0] + table_AO[channel][1];  

  if (dvalue >= 32767.) {
    value = 0x7fff;
  } else if (dvalue <= -32768.) {
    value = 0x8000;
  } else {
    value = (short int) dvalue;
  }

  memcpy(aOut.value, &value, 2);
  aOut.command = 0x10 | (channel << 1);
  
  if (usbAOutScanStatus_USB2416_4AO(udev, &depth) & OUTPUT_SCAN_RUNNING) {
    printf("There are currently %d samples in the Output FIFO buffer.\n", depth);
    return;
  }
  usb_control_msg(udev, requesttype, AOUT, 0x0, 0x0, (char *) &aOut, sizeof(aOut), HS_DELAY);
}


/***********************************************
 *          Miscellaneous Commands             *
 ***********************************************/

void usbCounterInit_USB2416(usb_dev_handle *udev, __u8 counter)
{
  /*
    This command initializes the 32-bit event counter.  On a write, the
     counter will be initialized to zero.
  */

  __u8 requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  usb_control_msg(udev, requesttype, COUNTER, 0x0, 0x0, (char *) &counter, sizeof(counter), HS_DELAY);
  
  return;
}

__u32 usbCounter_USB2416(usb_dev_handle *udev, __u8 counter)
{
  /*
    This command reads the 32-bit event counter.  
  */

  __u8 requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  __u32 counts[2] = {0x0, 0x0};

  usb_control_msg(udev, requesttype, COUNTER, 0x0, 0x0, (char *) &counts, sizeof(counts), HS_DELAY);
  if (counter == COUNTER0) {
    return counts[0];
  } else {
    return counts[1];
  }
}

void usbCJC_USB2416(usb_dev_handle *udev, float temp[8])
{
  /*
    This command reads the CJC sensors.  The temperature in degrees
    Celsius is calculated as:

     T = 128.(value/2^15)
  */
  __u8 requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  short int value[8];
  int i;

  usb_control_msg(udev, requesttype, CJC, 0x0, 0x0, (char *) &value, sizeof(value), HS_DELAY);
  for (i = 0; i < 8; i++) {
    temp[i] = value[i]/256.0;
  }
}

/* blinks the LED of USB device */
void usbBlink_USB2416(usb_dev_handle *udev, __u8 bcount)
{
  /*
    This command will blink the device LED "count" number of times
  */
  __u8 requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  __u8 cmd = BLINK_LED;

  printf("Blinking LED (%x) for %d counts\n", cmd, bcount);
  usb_control_msg(udev, requesttype, BLINK_LED, 0x0, 0x0, (char *) &bcount, 1, HS_DELAY);
  return;
}

__u8 usbStatus_USB2416(usb_dev_handle *udev)
{
  /*
    This command retrieves the status of the device.
  */
  __u8 requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  __u8 status = 0x0;

  usb_control_msg(udev, requesttype, GET_STATUS, 0x0, 0x0, (char *) &status, sizeof(status), HS_DELAY);
  return status;
}  

void usbGetSerialNumber_USB2416(usb_dev_handle *udev, char serial[9])
{
  /*
    This commands reads the device USB serial number.  The serial
    number consists of 8 bytes, typically ASCII numeric or hexadecimal digits
    (i.e. "00000001"). 
  */
  __u8 requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  usb_control_msg(udev, requesttype, SERIAL, 0x0, 0x0, serial, 8, HS_DELAY);
  serial[8] = '\0';
  return;
}

void usbSetSerialNumber_USB2416(usb_dev_handle *udev, char serial[9])
{
  /*
    This command writes the device USB serial number.  The serial
    number consists of 8 bytes, typically ASCII numeric or hexadecimal digits
    (i.e. "00000001"). The new serial number will be programmed but not used until
    hardware reset.
  */
  __u8 requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  usb_control_msg(udev, requesttype, SERIAL, 0x0, 0x0, serial, 8, HS_DELAY);
  return;
}

void usbGetVersion_USB2416(usb_dev_handle *udev, __u16 version[4])
{
  /*
    This command reads the microcontroller firmware versions.  The firmware
    versions are returned as packed hexadecmal BCD values, i.e. if version
    = 0x0132, then the firmware version is 1.32.
  */
  __u8 requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  usb_control_msg(udev, requesttype, VERSION, 0x0, 0x0, (char *) version, 8, HS_DELAY);
}

void usbReadMemory_USB2416(usb_dev_handle *udev, __u16 length,  __u16 address, __u8 *data)
{
  /* This command reads data from the available data EEPROM memory.
     The number of bytes to read is specified in the wLength (for
     writes it is wLength - sizeof(address)).
  */

  __u8 requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  usb_control_msg(udev, requesttype, MEMORY, address, 0x0, (char *) data, length, HS_DELAY);
  return;
}

void usbWriteMemory_USB2416(usb_dev_handle *udev, __u16 length,  __u16 address, __u8 *data)
{
  /* This command writes data to the available data EEPROM memory. 
     The number of bytes to read is specified in the wLength (for
     writes it is wLength - sizeof(address)).  The first 2 byes of data is
     the address.

     Note: this function is not reentrant
  */

  char *buf;
  __u8 requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  buf = malloc(length + 2);
  memcpy(buf, &address, 2);
  memcpy(&buf[2], data, length);
  usb_control_msg(udev, requesttype, MEMORY, 0x0, 0x0, buf, length+2, HS_DELAY);
  free(buf);
  return;
}

void usbReset_USB2416(usb_dev_handle *udev)
{
  /*
    This function causes the device to perform a reset.  The device disconnects from the USB bus and resets
    its microcontroller.
  */

  __u8 requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  usb_control_msg(udev, requesttype, RESET, 0x0, 0x0, NULL, 0, HS_DELAY);
  return;
}

void usbCalConfig_USB2416(usb_dev_handle *udev, __u8 value)
{
  /*
    This command will configure the calibration source.
    value =  0:  +0.078V
             1:  -0.078V
             2:  +0.156V
	     3:  -0.156V
	     4:  +0.325V
     	     5:  -0.325V
	     6:  +0.626V
     	     7:  -0.626V
	     8:  +1.25V
     	     9:  -1.25V
	    10:  +2.50V
     	    11:  -2.50V
	    12:  +5.00V
     	    13:  -5.00V
	    14:  +10.0V
     	    15:  -10.0V
	    16:  +18.0V
     	    17:  -18.0V
            18: External calibration source.	     
  */
  __u8 requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  usb_control_msg(udev, requesttype, CAL_CONFIG, 0x0, 0x0, (char *) &value, sizeof(value), HS_DELAY);
  return;
}

void usbADCal_USB2416(usb_dev_handle *udev)
{
  /*
    The command will perform A/D self calibration.
  */
  __u8 requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  usb_control_msg(udev, requesttype, ADCAL, 0x0, 0x0, NULL, 0x0, HS_DELAY);
  return;
}

void usbTCCalMeasure(usb_dev_handle *udev, __u8 value)
{
  /* The command will enable measurement of the TC cal source
     value: 0: normal operation
            1: TC cal source measurment mode (JP3)
  */
  __u8 requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  usb_control_msg(udev, requesttype, TC_CAL, 0x0, 0x0, (char *) &value, sizeof(value), HS_DELAY);
  return;
}

void cleanup_USB2416( usb_dev_handle *udev )
{
  if (udev) {
    usb_clear_halt(udev, USB_ENDPOINT_IN|1);
    usb_clear_halt(udev, USB_ENDPOINT_OUT|1);
    usb_release_interface(udev, 0);
    usb_close(udev);
  }
}

void voltsTos16_USB2416_4AO(double *voltage, __s16 *data, int nSamples, double table_AO[])
{
  /* This routine converts an array of voltages (-10 to 10 volts) to singed 24 bit ints for the DAC */
  int i;
  double dvalue;

  for (i = 0; i < nSamples; i++) {
    dvalue = voltage[i]*(1<<15)/10.;                             /* convert voltage to signed value */
    dvalue = dvalue*table_AO[0] + table_AO[1];                   /* correct for calibration errors */
    if (dvalue >= 32767.) {
      data[i] = 0x7fff;
    } else if (dvalue <= -32768.) {
      data[i] = 0x8000;
    } else {
      data[i] = (short int) dvalue;
    }
  }
}

double volts_USB2416(usb_dev_handle *udev, const int gain, const int value)
{
  double volt = 0.0;
  
  switch (gain) {
    case BP_20V:
      volt = value * 20.0 / 0x7fffff;
      break;
    case BP_10V:
      volt = value * 10.0 / 0x7fffff;
      break;
    case BP_5V:
      volt = value * 5.0 / 0x7fffff;
      break;
    case BP_2_5V:
      volt = value * 2.5 / 0x7fffff;
      break;
    case BP_1_25V:
      volt = value * 1.25 / 0x7fffff;
      break;
    case BP_625V:
      volt = value * 0.625 / 0x7fffff;
      break;
    case BP_312V:
      volt = value * 0.312 / 0x7fffff;
      break;
    case BP_156V:
      volt = value * 0.156 / 0x7fffff;
      break;
    case BP_078V:
      volt = value * 0.078 / 0x7fffff;
      break;
  }

  return volt;
}
