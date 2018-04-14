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

// Browse to the IP of this server to get the
// current time.

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <NTP.h>

// Available from github.com/PaulStoffregen/Time
#include <TimeLib.h>

char *ssid = "Your SSID";
const char *password = "Your PSK";

ESP8266WebServer server ( 80 );
NTP ntp("pool.ntp.org");

void handleRoot() {
    char temp[50];
    sprintf(temp, "%02d/%02d/%04d %02d:%02d:%02d",
        day(), month(), year(), 
        hour(), minute(), second()
    );
    server.send( 200, "text/plain", temp);
}

void handleNotFound() {
    server.send ( 404, "text/plain", "Not Found" );
}

void setup ( void ) {    
    Serial.begin ( 115200 );
    WiFi.mode ( WIFI_STA );
    WiFi.begin ( ssid, password );
    Serial.println ( "" );

    // Wait for connection
    while ( WiFi.status() != WL_CONNECTED ) {
        delay ( 500 );
        Serial.print ( "." );
    }

    Serial.println ( "" );
    Serial.print ( "Connected to " );
    Serial.println ( ssid );
    Serial.print ( "IP address: " );
    Serial.println ( WiFi.localIP() );

    server.on ( "/", handleRoot );
    server.onNotFound ( handleNotFound );
    server.begin();
    Serial.println ( "HTTP server started" );


    Serial.print("Starting NTP...");
    ntp.begin();
    Serial.println(ntp.getServerAddress());

    Serial.print("Querying time...");
    uint32_t secs;
    if (ntp.queryTime(secs)) {
        Serial.println(secs);
    } else {
        Serial.println("Failed");
    }
    
    // Set the time in TimeLib.
    setTime(secs);

    // This makes the TimeLib update the
    // time automatically     
    setSyncProvider(getNtpTime);
    setSyncInterval(60);
}

void loop ( void ) {
    server.handleClient();
}

// Callback that gets the time for
// TimeLib's auto-update.
time_t getNtpTime() {
    uint32_t t;
    Serial.print("Updating time...");
    ntp.queryTime(t);
    Serial.println(t);
    return t;
}