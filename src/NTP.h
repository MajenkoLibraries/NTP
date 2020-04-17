/*
 * Copyright (c) 2018, Majenko Technologies
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * * Neither the name of Majenko Technologies nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _NTP_H
#define _NTP_H

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

typedef struct NTPV4HDRTIME_T
{
    uint32_t    cSeconds;
    uint32_t    secFraction;
} NTPV4HDRTIME;                 // headers only use the 64 bit form


typedef struct NTPV4_T
{
    union
    {
        uint8_t         flags;              //  e3  11100011
        struct
        {
            unsigned    mode    : 3;
            unsigned    vn      : 3;
            unsigned    li      : 2;
        }__attribute__((packed)) ;
    }__attribute__((packed)) ;
    uint8_t             stratum;            //  00
    uint8_t             poll;               //  03
    int8_t              precision;          //  fa
    int32_t             rootDelay;          //  00010000
    uint32_t            rootDispersion;     //  00010000
    union
    {
        uint8_t         rgchRefId[4];
        uint32_t        ipSynSource;        //  00000000
    }__attribute__((packed)) ;
    NTPV4HDRTIME        refTimeStamp;       //  0000000000000000
    NTPV4HDRTIME        orgTimeStamp;       //  0000000000000000
    NTPV4HDRTIME        recTimeStamp;       //  0000000000000000
    NTPV4HDRTIME        transmitTimeStamp;  //  0000000000000000
//  uint32_t            Extension[];                // optional
//  uint32_t            keyId;                      // optional
//  uint128_t           messageDigest;              // optional
} __attribute__((packed)) NTPV4;

class NTP {
    private:
        WiFiUDP _udp;
        const char *_hostname;
        IPAddress _ip;
        uint32_t _currentTime;

    public:

        NTP(const char *host);

        void begin();
        bool sendQuery();
        bool responseReady();
        uint32_t getTime() { return _currentTime; }


        bool queryTime(uint32_t &secs);

        IPAddress &getServerAddress() { return _ip; }
};

#endif
