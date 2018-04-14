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

#include <NTP.h>

NTP::NTP(const char *n) {
    _hostname = n;
}

void NTP::begin() {
    WiFi.hostByName(_hostname, _ip);
}

bool NTP::sendQuery() {
    NTPV4 ts;

    uint32_t t = millis();
    while (_ip == (uint32_t)0x00000000UL) {
        WiFi.hostByName(_hostname, _ip);
        if (millis() - t > 1000) {
            return false;
        }
        yield();
    }

    _udp.begin(2390);

    memset(&ts, 0, sizeof(ts));

    ts.mode = 3; // Client
    ts.vn = 4; // NTPv4
    ts.li = 3;
    ts.poll = 3;
    ts.precision = 0xfa;
    ts.rootDelay = 0x00010000;
    ts.rootDispersion = 0x00010000;

    _udp.beginPacket(_ip, 123);
    _udp.write((uint8_t *)&ts, sizeof(ts));
    _udp.endPacket();
}

bool NTP::responseReady() {
    NTPV4 ts;
    if (!_udp.parsePacket()) {
        return false;
    }

    _udp.read((uint8_t *)&ts, sizeof(ts));
    _udp.stop();

    uint32_t secs = ts.recTimeStamp.cSeconds;
    _currentTime =  ((secs << 24) | ((secs & 0xff00) << 8) | ((secs & 0xFF0000) >> 8) | (secs >> 24)) - 2208988800UL;
    return true;
}

bool NTP::queryTime(uint32_t &secs) {
    if (!sendQuery()) {
        return false;
    }

    uint32_t ts = millis();
    while (!responseReady()) {
        if (millis() - ts > 1000) return false;
    }
    secs = getTime();
    return true;
}
