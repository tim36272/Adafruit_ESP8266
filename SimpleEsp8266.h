/*------------------------------------------------------------------------
An Arduino library for the ESP8266 WiFi-serial bridge

https://www.adafruit.com/product/2282

The ESP8266 is a 3.3V device.  Safe operation with 5V devices (most
Arduino boards) requires a logic-level shifter for TX and RX signals.

Adafruit invests time and resources providing this open source code,
please support Adafruit and open-source hardware by purchasing
products from Adafruit!

Written by Limor Fried and Phil Burgess for Adafruit Industries.
MIT license, all text above must be included in any redistribution.
------------------------------------------------------------------------*/

#ifndef _SimpleESP8266_H_
#define _SimpleESP8266_H_
//Override the serial buffer size in Arduino
//#undef SERIAL_RX_BUFFER_SIZE
//#define SERIAL_RX_BUFFER_SIZE 256
#include <Arduino.h>

#define ESP_RECEIVE_TIMEOUT   5000L     //Time (in milliseconds) to wait for generic responses from the device
#define ESP_RESET_TIMEOUT     5000L     //Time (in milliseconds) to wait for device to reboot during a soft reset
#define ESP_CONNECT_TIMEOUT   15000L    //Time (in milliseconds) to wait for access point associtation to complete
#define ESP_CLIENT_TIMEOUT    7200000L  //Time (in milliseconds) to wait for a TCP connection
#define ESP_DATA_TIMEOUT      7200000L  //Time (in milliseconds) to wait for data after TCP connection established

#ifdef _VMICRO_INTELLISENSE
    //The VMICRO environment doesn't have an accurate F definition, so replace it here
    #undef F
    #define F(string_literal) (reinterpret_cast<const __FlashStringHelper *>(PSTR(string_literal)))
#endif
typedef const __FlashStringHelper EspStr; // PROGMEM/flash-resident string

#define defaultBootMarker F("ready\r\n")

// Subclassing Print makes debugging easier -- output en route to
// WiFi module can be duplicated on a second stream (e.g. Serial).
class SimpleESP8266 : public Print
{
public:
    SimpleESP8266(Stream *stream = &Serial, Stream *debug = NULL, int8_t reset_pin = -1);
    boolean hardReset(void);
    boolean softReset(void);
    boolean find(EspStr *str = NULL, boolean ipd = false, boolean verbose = false);
    void setupUART(uint32_t baud = 115200, uint8_t data_bits = 8, uint8_t stop_bits = 1, uint8_t parity = 0, uint8_t flow_control = 0);
    boolean connectToAP(EspStr *ssid, EspStr *pass);
    boolean connectTCP(EspStr *host, int port);
    boolean acceptTCP(uint16_t port);
    boolean unacceptTCP();
    boolean requestURL(EspStr *url);
    boolean requestURL(char* url);
    int     readLine(char *buf, int buf_size);
    void    closeAP(void);
    void    closeTCP(void);
    void    debugLoop(void);
    void    setTimeouts(uint32_t receive_timeout = 0, 
                        uint32_t reset_timeout = 0, 
                        uint32_t ap_connect_timeout = 0, 
                        uint32_t client_timeout = 0, 
                        uint32_t data_timeout = 0);
    void    setDefaultTimeouts();
    void    setBootMarker(EspStr *s = NULL);
    void    clearStreamBuffer();
    void    setDebug(Stream *debug = NULL);

    //Most people will just want the function below: this will reset the device, set it up, and start a TCP server
    //Returns true if the server is waiting for data, false if an error ocurred.
    boolean setupTcpServer(EspStr *ssid, EspStr* password, uint16_t port = 80);
    int32_t tcpRecv(char *buffer, uint32_t buffer_len);
private:
    Stream    *stream_;     // -> ESP8266, e.g. SoftwareSerial or Serial1
    Stream    *debug_;      // -> host, e.g. Serial
    const char *indent_;      //all debug_ commands will be indented by this value
    uint32_t  receive_timeout_;
    uint32_t  reset_timeout_;
    uint32_t  connect_timeout_;
    uint32_t  client_timeout_;
    uint32_t  data_timeout_;
    int8_t    reset_pin_;  // -1 if RST not connected
    EspStr    *host_;       // Non-NULL when TCP connection open
    const EspStr    *boot_marker_; // String indicating successful boot
    boolean   writing_;
    virtual size_t write(uint8_t);
    void     escapedDebugPrint(char* str);
    void     escapedDebugWrite(char c);
};

#endif // _SimpleESP8266_H_
