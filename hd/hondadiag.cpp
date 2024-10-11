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

// #define DEBUG_MESSAGES
// #define TESTS

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
DESCRIPTION:
    Request = 72 AA BB CC CS
    72 = Request Header Code
    AA = Number of Bytes (including the Checksum)
    BB = Query Table
    CC = Table
    CS = Checksum

CHECKSUM

    Usually to make a data table can be started from 00 to FF. With code 72 05
71 ZZ CS Where ZZ = data table 00 to FF and CS = Checksum The formula checksum =
100 - (sumbyte and FF) For example the data request Table 13 = 72 05 71 13 CS
Then the checksum value = 100 - ((72 + 05 + 71 + 13) AND FF) , result CS = 05 So
the data request table 13 = 72 05 71 13 05


*/
#define HONDA_MAX_DATASIZE 100

typedef struct {
  uint8_t hrc;
  uint8_t cmd_len;
  uint8_t cmd[HONDA_MAX_DATASIZE];
} HONDA_PACKET;

const int HONDA_PROPRIETARY_TINIL = 70;
const int HONDA_PROPRIETARY_TWUP = 200; //~120ms

const HONDA_PACKET HELLO = {0x60, 0x02, {0x70, 0x02}};
const HONDA_PACKET GET_ECU_INFO = {0x60, 0x02, {0x20, 0xf}};
const HONDA_PACKET GET_ECU_SERIAL = {0x60, 0x02, {0x30, 0xf}};
const HONDA_PACKET GET_DTC[5] = {{0x60, 0x02, {0x08, 0x06}},
                                 {0x60, 0x02, {0x0A, 0x02}},
                                 {0x60, 0x02, {0x0C, 0x02}}};
const HONDA_PACKET CLR_ERR = {0x61, 0x01, {0x01}};
//
char szOut[1024]; // output string

void usage() { printf("Diagnostics of HONDA CR-V 3 SRS ECU.\n\n"); }

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

void dump_hp(HONDA_PACKET *hp) {
  printf("Packet : %02X [", hp->hrc);
  for (unsigned int i = 0; i < hp->cmd_len; i++)
    printf("%02X ", hp->cmd[i]);
  printf("]\n");
} //..dump_hp

uint8_t iso_checksum(uint8_t *data, uint16_t len) {
  uint8_t crc = 0;
  for (uint8_t i = 0; i < len; i++)
    crc = crc + data[i];
  return 0x100 - crc;
}

void make_packet(const HONDA_PACKET *cmd, PASSTHRU_MSG *msg) {
  memset(msg->Data, 0, 100);
  msg->Data[0] = cmd->hrc;
  uint8_t alen = cmd->cmd_len + 3;
  msg->Data[1] = alen;
  memcpy(msg->Data + 2, cmd->cmd, cmd->cmd_len);
  msg->Data[alen - 1] = iso_checksum(msg->Data, alen - 1);
  msg->DataSize = alen;
} //..make_packet

int decode_packet(PASSTHRU_MSG *msg, HONDA_PACKET *cmd) {
  int br = 1;
  cmd->hrc = msg->Data[0];
  cmd->cmd_len = msg->DataSize - 3;
  uint8_t cs = iso_checksum(msg->Data, msg->DataSize - 1);
  if (cs != msg->Data[msg->DataSize - 1]) {
    printf("Invalid checksum in result!"); // << warning
    br = 0;
  }
  if (cmd->cmd_len > HONDA_MAX_DATASIZE)
    cmd->cmd_len = HONDA_MAX_DATASIZE;
  memcpy(cmd->cmd, msg->Data + 2, cmd->cmd_len);
  return br;
} //..decode_packet

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

const char *dtc_fromdata(HONDA_PACKET *hp) {
  sprintf(szOut, "%02X-%02X", hp->cmd[0], hp->cmd[1]);
  return szOut;
} //..dtc_fromdata

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

int receivemsg(HONDA_PACKET *hp) {
  unsigned long numRxMsg;
  PASSTHRU_MSG rxmsg;
  memset(&rxmsg, 0, sizeof(rxmsg));
  time_t last_status_update = time(NULL);
  numRxMsg = 1;
  while (time(NULL) - last_status_update < 5) {
    if (0 == j2534.PassThruReadMsgs(chanID, &rxmsg, &numRxMsg, 1000)) {
      if (numRxMsg && rxmsg.DataSize) {
#ifdef DEBUG_MESSAGES
        dump_msg(&rxmsg); // debug
#endif
        decode_packet(&rxmsg, hp);
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

int sendmsg(const HONDA_PACKET *hp) {

  PASSTHRU_MSG txmsg, rxmsg;
  unsigned long NumMsgs = 1;

  txmsg.ProtocolID = ISO9141_K;
  txmsg.RxStatus = 0;
  txmsg.TxFlags = 0;
  txmsg.Timestamp = time(NULL);
  txmsg.DataSize = 0;
  txmsg.ExtraDataIndex = 0;
  make_packet(hp, &txmsg);
#ifdef DEBUG_MESSAGES
  dump_msg(&txmsg); // debug
#endif
  if (g_FirstMessage) {
    g_FirstMessage = 0;
    if (j2534.PassThruIoctl(chanID, FAST_INIT, &txmsg, &rxmsg)) {
      // printf("Error sending hello message\n");
      //   return 0;
    }
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

  usage();

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

  // printf("J2534 API Version: %s\n", strApiVersion);
  // printf("J2534 DLL Version: %s\n", strDllVersion);
  // printf("Device Firmware Version: %s\n", strFirmwareVersion);
  // printf("Device Serial Number: %s\n", strSerial);

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
  // printf("Configured successfully...\n");

  PASSTHRU_MSG rxmsg, txmsg;

  // now setup the filter(s)
  PASSTHRU_MSG msgMask, msgPattern;
  unsigned long msgId;
  unsigned long numRxMsg;
  HONDA_PACKET hpRec; //!< recieve packet
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

  printf("Start-up communication...\n");
  HONDA_PACKET hp = HELLO;
  sendmsg(&hp);
  receivemsg(&hpRec);
#ifndef TESTS
  printf("Reading ECU information...\n");
  hp.cmd[1] = 0x0F;
  sendmsg(&hp);
  receivemsg(&hpRec);
  printf("SOME ID: %s\n", hextostr(hpRec.cmd, hpRec.cmd_len));

  hp = GET_ECU_INFO;
  sendmsg(&hp);
  receivemsg(&hpRec);
  printf("ECU ID: %s\n", hpRec.cmd);

  hp = GET_ECU_SERIAL;
  sendmsg(&hp);
  receivemsg(&hpRec);
  printf("ECU SERIAL: %s\n", hpRec.cmd);

  printf("Reading DTC information...\n");
  for (int i = 0; i < 3; i++) {
    sendmsg(&GET_DTC[i]);
    receivemsg(&hpRec);
    printf("DTC: %s\n", dtc_fromdata(&hpRec));
  }

  printf("Clearing DTC information...\n");
  sendmsg(&CLR_ERR);
  receivemsg(&hpRec);
  printf("Clear: %s\n", hextostr(hpRec.cmd, hpRec.cmd_len));

#else

  HONDA_PACKET sp;
  for (int i = 0; i < 0xFF; i++) {
    for (int inter = 0x5F; inter < 0x63; inter++) {
      sp.hrc = inter; //!< we know only this interface and 0x61
      for (int j = 2; j < 0x0f; j++) {
        sp.cmd_len = j;
        sp.cmd[1] = i;
        sp.cmd[0] = i;
        printf("Sending >");
        dump_hp(&sp);
        sendmsg(&sp);
        receivemsg(&hpRec);
        printf("Reply < %s\n", hextostr(hpRec.cmd, hpRec.cmd_len));
      } //..j
    }   //..inter
  }     //..i
#endif

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
