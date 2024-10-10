
//
// Copyright (C) 2004- Tactrix Inc.
//
// You are free to use this file for any purpose, but please keep
// notice of where it came from!
//
//////////////////////////////////////////////////////////////////////////////

#include "../common/J2534.h"
#include <conio.h>
#include <iostream>
#include <stdio.h>
#include <tchar.h>
#include <time.h>
#include <windows.h>

#define MIN(a, b) (((a) < (b)) ? (a) : (b))

/*
HONDA KEIHIN KLINE PROTOCOL (DIAGNOSTIC)
    Pin Tx = Low (70 ms)
    Pin Tx = High (120 ms)
    Set Baudrate = 10400 bps
    Wake Up with send to ECU code = FE 04 72 8C
    ECU Response with code = 0E 04 72 7C
    Send Initialise code = 72 05 00 F0 99
    ECU Response = 02 04 00 FA
    Repeat point 1 if there is no response from the ECU. And continue the next
Steps if there is a response from the ECU

*/
const int HONDA_PROPRIETARY_TINIL = 70;
const int HONDA_PROPRIETARY_TWUP = 200; //~120ms
const uint8_t HELLO[] = {0x60, 0x05, 0x70, 0x02, 0x29};
const uint8_t GET_ECU_INFO[] = {0x60, 0x05, 0x70, 0x0F, 0x1C};
//                              ^^id  ^^len ^^ data
const uint8_t GET_DTC[][5] = {{0x60, 0x05, 0x08, 0x02, 0x91},
                              {0x60, 0x05, 0x0A, 0x02, 0x8F},
                              {0x60, 0x05, 0x0C, 0x02, 0x8D}};

const uint8_t CLR_ERR[] = {0x61, 0x04, 0x01, 0x9A};
char szOut[1024];

void usage() {
  printf("Diagnostics of HONDA CR-V 3 SRS ECU.\n\n");
  exit(0);
}

J2534 j2534;
unsigned long devID;
unsigned long chanID;

void reportJ2534Error() {
  char err[512];
  j2534.PassThruGetLastError(err);
  printf("J2534 error [%s].", err);
}

void dump_msg(PASSTHRU_MSG *msg) {
  if (msg->RxStatus & START_OF_MESSAGE)
    return; // skip

  printf("[%u] ", msg->Timestamp);
  for (unsigned int i = 0; i < msg->DataSize; i++)
    printf("%02X ", msg->Data[i]);
  printf("\n");
} //..dump_msg

const char *hextostr(uint8_t *ptr, int size) {
  char szHex[5];
  szOut[0] = 0;
  for (unsigned int i = 0; i < size; i++) {
    sprintf(szHex, "%02X ", ptr[i]);
    strcat(szOut, szHex);
  } //..for
  // szOut[size] = 0;
  return szOut;
} //..dump_msg

const char *dtc_fromdata(uint8_t *ptr, int size) {
  if (size < 4)
    return "";
  sprintf(szOut, "%02X-%02X", ptr[2], ptr[3]);
  // szOut[size] = 0;
  return szOut;
}

bool get_serial_num(char *serial) {
  struct {
    unsigned int length;
    unsigned int svcid;
    unsigned short infosvcid;

  } inbuf;

  struct {
    unsigned int length;
    unsigned char data[256];
  } outbuf;

  inbuf.length = 2;
  inbuf.svcid = 5;     // info
  inbuf.infosvcid = 1; // serial

  outbuf.length = sizeof(outbuf.data);

  if (j2534.PassThruIoctl(devID, TX_IOCTL_APP_SERVICE, &inbuf, &outbuf)) {
    serial[0] = 0;
    return false;
  }

  memcpy(serial, outbuf.data, outbuf.length);
  serial[outbuf.length] = 0;
  return true;
}

int receivemsg(uint8_t *pmsg, uint16_t validsize, uint16_t *msglen) {
  unsigned long numRxMsg;
  PASSTHRU_MSG rxmsg;
  memset(&rxmsg, 0, sizeof(rxmsg));
  time_t last_status_update = time(NULL);
  numRxMsg = 1;
  while (time(NULL) - last_status_update < 5) {
    if (0 == j2534.PassThruReadMsgs(chanID, &rxmsg, &numRxMsg, 1000)) {
      if (numRxMsg && rxmsg.DataSize) {
        // dump_msg(&rxmsg); // debug
        *msglen = MIN(validsize, rxmsg.DataSize);
        memcpy(pmsg, rxmsg.Data, *msglen);
        break;
      }
    }
  }
  return numRxMsg;
} //..receivemsg

int g_FirstMessage = 1;

/*

7.3.6 FAST_INIT
The IoctlID value of FAST_INIT is used to initiate a fast initialization
sequence from the pass-thru device. The calling application is responsible for
allocating and initializing the associated parameters described in Figure 34.
When the function is successfully completed, the response message will be placed
in structure pointed to by OutputPtr. It should be noted that this only applies
to Protocol ID of ISO9141 or ISO14230. In case of successful initialization the
OutputPtr will contain valid data

Honda uses an undocumented proprietary protocol over K-Line at 10400 baud.
A pulse of 70 ms similar to the Fast Init of ISO14230 is sent to wake up the ECU
before communication starts.
*/

int sendmsg(const uint8_t *msg, int msglen) {

  PASSTHRU_MSG txmsg, rxmsg;
  unsigned long NumMsgs = 1;

  txmsg.ProtocolID = ISO9141_K;
  txmsg.RxStatus = 0;
  txmsg.TxFlags = 0;
  txmsg.Timestamp = time(NULL);
  txmsg.DataSize = msglen;
  txmsg.ExtraDataIndex = 0;

  memcpy(txmsg.Data, msg, msglen);
  memcpy(rxmsg.Data, msg, msglen);
  printf("Sending %d: ", msglen);
  // dump_msg(&txmsg);
  if (g_FirstMessage) {
    g_FirstMessage = 0;
    if (j2534.PassThruIoctl(chanID, FAST_INIT, &txmsg, &rxmsg)) {
      // printf("Error sending hello message\n");
      //   return 0;
    }
    // dump_msg(&rxmsg);
  } else
    return j2534.PassThruWriteMsgs(chanID, &txmsg, &NumMsgs, 0);

  return 0;
} //..sendmsg

int _tmain(int argc, _TCHAR *argv[]) {
  char *outfile = NULL;
  unsigned int protocol = ISO9141_K;
  unsigned int baudrate = 10400;
  unsigned int parity = NO_PARITY;
  unsigned int timeout = 50;

  for (int argi = 1; argi < argc; argi++) {
    if (argv[argi][0] == '/' || argv[argi][0] == '-') {
      // looks like a switch
      char *sw = &argv[argi][1];
      if (strcmp(sw, "d") == 0) {
        argi++;
        if (argi >= argc)
          usage();

      } else if (strcmp(sw, "c") == 0) {

      } else if (strcmp(sw, "b") == 0) {
      }
    }

  } //..for

  if (!j2534.init()) {
    printf("can't connect to J2534 DLL.\n");
    return 0;
  }

  if (j2534.PassThruOpen(NULL, &devID)) {
    reportJ2534Error();
    return 0;
  }

  char strApiVersion[256];
  char strDllVersion[256];
  char strFirmwareVersion[256];
  char strSerial[256];

  if (j2534.PassThruReadVersion(strApiVersion, strDllVersion,
                                strFirmwareVersion, devID)) {
    reportJ2534Error();
    return false;
  }

  if (!get_serial_num(strSerial)) {
    reportJ2534Error();
    return false;
  }

  printf("J2534 API Version: %s\n", strApiVersion);
  printf("J2534 DLL Version: %s\n", strDllVersion);
  printf("Device Firmware Version: %s\n", strFirmwareVersion);
  printf("Device Serial Number: %s\n", strSerial);

  // use ISO9141_NO_CHECKSUM to disable checksumming on both tx and rx
  // messages
  if (j2534.PassThruConnect(devID, protocol,
                            ISO9141_K_LINE_ONLY | ISO9141_NO_CHECKSUM, baudrate,
                            &chanID)) {
    reportJ2534Error();
    return 0;
  }

  // set timing

  SCONFIG_LIST scl;
  SCONFIG scp[4] = {{P1_MAX, 0}, {PARITY, 0}, {TWUP, 50}, {TINIL, 25}};
  scl.NumOfParams = 4;
  scp[0].Value = timeout * 2;
  scp[1].Value = parity;
  scp[2].Value = HONDA_PROPRIETARY_TWUP;
  scp[3].Value = HONDA_PROPRIETARY_TINIL;
  scl.ConfigPtr = scp;
  if (j2534.PassThruIoctl(chanID, SET_CONFIG, &scl, NULL)) {
    reportJ2534Error();
  }
  printf("Configured successfully...\n");

  PASSTHRU_MSG rxmsg, txmsg;

  // now setup the filter(s)
  PASSTHRU_MSG msgMask, msgPattern;
  unsigned long msgId;
  unsigned long numRxMsg;
  uint8_t recMsg[100];
  uint16_t l;

  // simply create a "pass all" filter so that we can see
  // everything unfiltered in the raw stream

  txmsg.ProtocolID = protocol;
  txmsg.RxStatus = 0;
  txmsg.TxFlags = 0;
  txmsg.Timestamp = 0;
  txmsg.DataSize = 1;
  txmsg.ExtraDataIndex = 0;
  msgMask = msgPattern = txmsg;
  memset(msgMask.Data, 0, 1);    // mask the first 4 byte to 0
  memset(msgPattern.Data, 0, 1); // match it with 0 (i.e. pass everything)
  if (j2534.PassThruStartMsgFilter(chanID, PASS_FILTER, &msgMask, &msgPattern,
                                   NULL, &msgId)) {
    reportJ2534Error();
    return 0;
  }

  sendmsg(HELLO, sizeof(HELLO));
  printf("11111111111\n");
  receivemsg(recMsg, 100, &l);
  sendmsg(GET_ECU_INFO, sizeof(GET_ECU_INFO));
  receivemsg(recMsg, 100, &l);
  printf("ECU ID: %s\n", hextostr(recMsg, l));

  for (int i = 0; i < 3; i++) {
    sendmsg(GET_DTC[i], sizeof(GET_DTC[i]));
    receivemsg(recMsg, 100, &l);
    printf("DTC: %s\n", dtc_fromdata(recMsg, l));
  }

  sendmsg(CLR_ERR, sizeof(CLR_ERR));
  receivemsg(recMsg, 100, &l);
  printf("Clear: %s\n", hextostr(recMsg, l));

  // shut down the channel

  if (j2534.PassThruDisconnect(chanID)) {
    reportJ2534Error();
    return 0;
  }

  // close the device

  if (j2534.PassThruClose(devID)) {
    reportJ2534Error();
    return 0;
  }
} //..main
