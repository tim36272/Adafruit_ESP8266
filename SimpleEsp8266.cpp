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

#include "SimpleESP8266.h"

//#define DEBUG_ENABLED
#ifdef DEBUG_ENABLED
#define DEBUG_STR(the_str) F(the_str)
#else
const char no_debug_string[] PROGMEM  = "DBGOFF";
#define DEBUG_STR(the_str) no_debug_string
#endif

//This type is needed because a function which takes an F() string (i.e.
//  PROGMEM-resident string) cannot be directly passed into functions such as 
//  strlen_P. Casting the value to Pchr allows strlen_P to read the value
typedef const PROGMEM char        Pchr;

// Constructor
SimpleESP8266::SimpleESP8266(Stream *stream, Stream *debug, int8_t reset_pin) :
    stream_(stream), debug_(debug), reset_pin_(reset_pin), host_(NULL), writing_(false)
{
    setDefaultTimeouts();
    indent_ = "  ";
};

// Override various timings.  Passing 0 for an item keeps current setting.
void SimpleESP8266::setTimeouts(uint32_t receive_timeout,
                    uint32_t reset_timeout,
                    uint32_t ap_connect_timeout,
                    uint32_t client_timeout,
                    uint32_t data_timeout)
{
    if (receive_timeout)
    {
        stream_->setTimeout(receive_timeout);
        //debug_->print(F("Changing receive_timeout timeout from "));
        //debug_->print(receiveTimeout_);
        //debug_->print(F(" to "));
        //debug_->println(receive_timeout);
        receive_timeout_ = receive_timeout;
        //debug_->print(indent_);
    }
    if (reset_timeout)
    {
        reset_timeout_ = reset_timeout;
    }
    if (ap_connect_timeout)
    {
        connect_timeout_ = ap_connect_timeout;
    }
    if (client_timeout)
    {
        //The device only allows timeouts up to 7200 seconds
        if (client_timeout > 7200000)
        {
            client_timeout = 7200000;
        }
        client_timeout_ = client_timeout;
    }
    if (data_timeout)
    {
        data_timeout_ = data_timeout;
        //debug_->print(F("Set data timeout to "));
        //debug_->println(data_timeout_);
    }
}
void SimpleESP8266::setDefaultTimeouts()
{
    stream_->setTimeout(ESP_RECEIVE_TIMEOUT);
    receive_timeout_ = ESP_RECEIVE_TIMEOUT;
    reset_timeout_ = ESP_RESET_TIMEOUT;

    connect_timeout_ = ESP_CONNECT_TIMEOUT;
    //The device only allows timeouts up to 7200 seconds
    if (ESP_CLIENT_TIMEOUT > 7200000)
    {
        client_timeout_ = 7200000;
    } else
    {
        client_timeout_ = ESP_CLIENT_TIMEOUT;
    }
    data_timeout_ = ESP_DATA_TIMEOUT;
}

// Anything printed to the EPS8266 object will be split to both the WiFi
// and debug_ streams.  Saves having to print everything twice in debug_ code.
size_t SimpleESP8266::write(uint8_t c)
{
    if (!writing_)
    {
        //the module often falls behind if we transmit too fast, so before starting any transmission sleep for a bit
        delay(10);
        writing_ = true;
        if (debug_)
        {
            debug_->print(DEBUG_STR("\r\n"));
            debug_->print(indent_);
            debug_->print(DEBUG_STR("-S->"));
        }
    }
    if (debug_)
    {
        escapedDebugWrite(c);
    }
    return stream_->write(c);
}

void SimpleESP8266::clearStreamBuffer()
{
    delay(250);
    while (stream_->available())
    {
        (void)stream_->read();
    }
}

void SimpleESP8266::setDebug(Stream *debug)
{
    debug_ = debug;
}

// Equivalent to Arduino Stream find() function, but with search string in
// flash/PROGMEM rather than RAM-resident.  Returns true if string found
// (any further pending input remains in stream_), false if timeout occurs.
// Can optionally pass NULL (or no argument) to read/purge the OK+CR/LF
// returned by most AT commands.  The ipd flag indicates this call follows
// a CIPSEND request and might be broken into multiple sections with +IPD
// delimiters, which must be parsed and handled (as the search string may
// cross these delimiters and/or contain \r or \n itself).
#define FIND_BUFFER_SIZE 8
boolean SimpleESP8266::find(EspStr *search_str, boolean ipd, boolean verbose)
{
    uint8_t  stringLength, matchedLength = 0;
    int      c;
    boolean  found = false;
    uint32_t tLastGoodData;
    uint16_t bytesAvailable;
    uint16_t bytesRead;
    boolean timedOut = false;
    boolean ipd_fail = false;
    char buffer[FIND_BUFFER_SIZE + 1]; //+1 to allow for nullchar

    if (search_str == NULL)
    {
        search_str = F("OK\r\n");
    }
    stringLength = strlen_P((Pchr*)search_str);

    if (debug_ && writing_)
    {
        debug_->println(DEBUG_STR("<-S-"));
    }
    writing_ = false;

    if (debug_)
    {
        debug_->print(indent_);
        debug_->print(DEBUG_STR("Search for: '"));
        uint8_t strpos = 0;
        char next_char = 1;
        while (next_char != '\0')
        {
            next_char = pgm_read_byte((Pchr *)search_str + strpos);
            strpos++;
            escapedDebugWrite(next_char);
        }
        debug_->print(DEBUG_STR("'..."));
        debug_->flush();
    }

    // Expecting next IPD marker?
    if (ipd)
    {
        // Find marker in stream_
        //"IPD" is the prefix for "I received the following data from the network".
        //  It is formatted as: +IPD,<ID>,<len>[,<remote IP>,<remote port>]:data"
        //  The code below grabs all the data from "+IPD" through the colon, i.e. it advances the data pointer to the data portion
        //print a newline to debug_ so that the formatting comes out correctly
        if (debug_)
        {
            debug_->println();
        }
        if (find(F("+IPD,")))
        {
            //The device formats the message like this:
            //  +IPD,#,DATALEN:DATA
            //  Where # is normally 0.
            //Scan until we get a : symbol (end of IPD, start of data)
            boolean result = find(F(":"));
            if (!result)
            {
                return result;
            }
        }
    }
    tLastGoodData = millis();
    while (!found)
    {

        //If the caller gave "\0" as the string to find, then this function was
        //  only suppose to search for the +IPD sequence, so at this point we
        //  can break out of the loop
        if (stringLength == 0)
        {
            found = true;
            break;
        }

        // Check for new data
        bytesAvailable = stream_->available();
        if (bytesAvailable > 0)
        {
            //Get all of the available data
            if (bytesAvailable > FIND_BUFFER_SIZE)
            {
                bytesAvailable = FIND_BUFFER_SIZE;
            }
            //Only read enough bytes to finish this message (so that we don't consume any data used by the following line)
            if (bytesAvailable > (stringLength - matchedLength))
            {
                bytesAvailable = (stringLength - matchedLength);
            }
            bytesRead = stream_->readBytes(buffer, bytesAvailable);
            if (bytesRead != bytesAvailable && debug_)
            {
                debug_->print(DEBUG_STR("Received "));
                debug_->print(bytesRead);
                debug_->print(DEBUG_STR(" bytes, expected "));
                debug_->println(bytesAvailable);

            }
            //null terminate the string for easy printing
            buffer[bytesRead] = '\0';
            if (debug_ && verbose)
            {
                debug_->print(indent_);
                debug_->print(DEBUG_STR("Got "));
                debug_->print(bytesAvailable);
                debug_->print(DEBUG_STR(" bytes: "));
                debug_->println(buffer);
            }
            //Compare the received buffer to the requested string
            for (uint8_t buffer_index = 0; buffer_index < bytesAvailable; ++buffer_index)
            {
                c = buffer[buffer_index];
                // Match next byte?
                if (c == pgm_read_byte((Pchr *)search_str +
                                       matchedLength))
                {
                    matchedLength++;
                    // Matched whole string?
                    if (matchedLength == stringLength)
                    {
                        //The string was fully matched
                        found = true;
                        break;
                    }
                } else
                {
                    // Character mismatch; reset match pointer+counter
                    matchedLength = 0;
                }
            }
            // Timeout resets w/each successfuly receive
            tLastGoodData = millis();
        } else
        {
            // No data on stream_, check for timeout
            if ((millis() - tLastGoodData) > receive_timeout_)
            {
                timedOut = true;
                break;
            }
        }
    }

    if (debug_)
    {
        if (found)
        {
            debug_->println(DEBUG_STR("found"));
        } else if (timedOut)
        {
            debug_->println(DEBUG_STR("not found (timeout)"));
        } else if (ipd_fail) {
            //don't print anything, it was already done
        } else
        {
            debug_->println(DEBUG_STR("not found (unspecified)"));
        }
    }

    return found;
}

void SimpleESP8266::setupUART(uint32_t baud, uint8_t data_bits, uint8_t stop_bits, uint8_t parity, uint8_t flow_control)
{
    stream_->print(F("AT+UART_CUR="));
    stream_->print(baud);
    stream_->print(F(","));
    stream_->print(data_bits);
    stream_->print(F(","));
    stream_->print(stop_bits);
    stream_->print(F(","));
    stream_->print(parity);
    stream_->print(F(","));
    stream_->println(flow_control);

    //Note that since this command changes the baud rate it doesn't really make sense to flush the buffer. So, the user should call clearStreamBuffer() after adjusting their own serial port
}

// Read from ESP8266 stream into RAM, up to a given size.  Max number of
// chars read is 1 less than this, so NUL can be appended on string.
int SimpleESP8266::readLine(char *buf, int buf_size)
{
    int bytesRead = 1;
    if (debug_ && writing_)
    {
        debug_->println(DEBUG_STR("<-S-"));
    }
    writing_ = false;
    buf[0] = '\0';
    //Ignore blank lines
    while (bytesRead <= 2 && (buf[0] == '\r' || buf[0] == '\n' || buf[0] == '\0'))
    {
        bytesRead = stream_->readBytesUntil('\n', buf, buf_size - 1);
    }
    buf[bytesRead] = 0;
    if (debug_)
    {
        debug_->print(indent_);
        debug_->print(DEBUG_STR("-R->"));
        escapedDebugPrint(buf);
        debug_->println(DEBUG_STR("<-R-"));
    }
    return bytesRead;
}

void SimpleESP8266::escapedDebugWrite(char c)
{
    if (debug_)
    {
        if (c == '\r')
        {
            debug_->write("\\r");
        } else if (c == '\n')
        {
            debug_->write("\\n");
        } else
        {
            debug_->write(c);
        }
    }
}

void SimpleESP8266::escapedDebugPrint(char* str)
{
    uint16_t strpos = 0;
    while (str[strpos] != '\0')
    {
        escapedDebugWrite(str[strpos]);
        strpos++;
    }
}

// ESP8266 is reset by momentarily connecting RST to GND.  Level shifting is
// not necessary provided you don't accidentally set the pin to HIGH output.
// It's generally safe-ish as the default Arduino pin state is INPUT (w/no
// pullup) -- setting to LOW provides an open-drain for reset.
// Returns true if expected boot message is received (or if RST is unused),
// false otherwise.
boolean SimpleESP8266::hardReset(void)
{
    boolean found;
    if (reset_pin_ < 0)
    {
        return true;
    }
    digitalWrite(reset_pin_, LOW);
    pinMode(reset_pin_, OUTPUT); // Open drain; reset -> GND
    delay(10);                  // Hold a moment
    pinMode(reset_pin_, INPUT);  // Back to high-impedance pin state
    found = find((EspStr*)defaultBootMarker);    // Purge boot message from stream_
                                 //Discard any remaining bytes in the stream_
    clearStreamBuffer();
    return found;
}

// Soft reset.  Returns true if expected boot message received, else false.
boolean SimpleESP8266::softReset(void)
{
    boolean  found = false;
    uint32_t save = receive_timeout_; // Temporarily override recveive timeout,
    setTimeouts(reset_timeout_);    // reset time is longer than normal I/O.
    this->println(F("AT+RST"));            // Issue soft-reset command
    // Wait for boot message
    if (find((EspStr*)defaultBootMarker))
    {
        //Wait for any other post-boot messages
        delay(1000);
        clearStreamBuffer();
        if (debug_)
        {
            debug_->print(indent_);
            debug_->print(DEBUG_STR("Echo off"));
        }
        this->println(F("ATE0"));            // Turn off echo
        found = find();                // OK?
    }
    setTimeouts(save);   // Restore normal receive timeout
    //Discard any remaining bytes in the stream_ (for example if it automatically connects to WiFi
    clearStreamBuffer();
    return found;
}

// For interactive debugging...shuttle data between Serial Console <-> WiFi
void SimpleESP8266::debugLoop(void)
{
    // If no debug connection, nothing to do.
    if (!debug_)
    {
        return;
    }

    debug_->println(DEBUG_STR("\n="));
    for (;;)
    {
        if (debug_->available())
        {
            stream_->write(debug_->read());
        }
        if (stream_->available())
        {
            debug_->write(stream_->read());
        }
    }
}

// Connect to WiFi access point.  SSID and password are flash-resident
// strings.  May take several seconds to execute, this is normal.
// Returns true on successful connection, false otherwise.
boolean SimpleESP8266::connectToAP(EspStr *ssid, EspStr *pass)
{
    clearStreamBuffer();
    this->println(F("AT+CWMODE=1")); // WiFi mode = Sta
    if (!find())
    {
        return false;
    }
    this->print(F("AT+CWJAP=\"")); // Join access point
    this->print(ssid);
    this->print(F("\",\""));
    this->print(pass);
    this->println(F("\""));
    uint32_t save = receive_timeout_;  // Temporarily override recv timeout,
    setTimeouts(connect_timeout_); // connection time is much longer!
    boolean found = find();          // Await 'OK' message
    setTimeouts(save);           // Restore normal receive timeout
    if (found)
    {
        if (debug_)
        {
            debug_->print(indent_);
            debug_->println(DEBUG_STR("Associated with AP"));
        }
        this->println(F("AT+CIPMUX=0"));     // Set single-client mode
        found = find();                // Await 'OK'
        if (debug_)
        {
            debug_->print(indent_);
            debug_->println(DEBUG_STR("Set to single-client mode"));
        }
    }
    return found;
}

void SimpleESP8266::closeAP(void)
{
    this->println(F("AT+CWQAP")); // Quit access point
    find(); // Purge 'OK'
}

// Open TCP connection to an already-listening host.  Hostname is flash-resident string.
// Returns true on successful connection, else false.
boolean SimpleESP8266::connectTCP(EspStr *hostname, int port)
{

    this->print(F("AT+CIPSTART=\"TCP\",\""));
    this->print(hostname);
    this->print(F("\","));
    this->println(port);

    if (find())
    {
        host_ = hostname;
        return true;
    }
    return false;
}

// Accept TCP connection.
// Returns true on successful setup, else false.
boolean SimpleESP8266::acceptTCP(uint16_t port)
{
    this->println(F("AT+CIPMODE=0"));
    if (!find())
    {
        return false;
    }
    this->println(F("AT+CIPMUX=1"));
    if (!find())
    {
        return false;
    }

    this->print(F("AT+CIPSERVER=1,"));
    this->println(port);

    if (!find())
    {
        return false;
    }
    this->print(F("AT+CIPSTO="));
    this->println(client_timeout_ / 1000);
    if (!find())
    {
        return false;
    }

    return true;
}
int32_t SimpleESP8266::tcpRecv(char *buffer, uint32_t buffer_len)
{
    uint32_t buffer_pos = 0; //index of the next character to write to
    int32_t bytes_available;
    //Check here first for bytes available, and return if they are not
    uint32_t t0 = millis();
    while (!stream_->available())
    {
        if (millis() - t0 > data_timeout_)
        {
            return -1;
        }
    }
    //Wait for connection
    if (!find(F(""), true))
    {
        return -1;
    }
    //Dequeue all pending data to the buffer
    bytes_available = stream_->available();
    if (bytes_available == 0)
    {
        //The data may have just not arrived yet, so sleep a bit and check again
        //At a baud rate of 115200 there is 0.0694 milliseconds between
        //  characters, and even at 9600 baud there is 0.8 milliseconds,
        //  so sleeping for 1 millisecond should be plenty long enough.
        delay(1);
        bytes_available = stream_->available();
    }
    while (bytes_available > 0 && buffer_pos < buffer_len)
    {
        if (bytes_available > buffer_len - buffer_pos)
        {
            bytes_available = buffer_len - buffer_pos;
        }
        buffer_pos += stream_->readBytes(buffer + buffer_pos, bytes_available);
        bytes_available = stream_->available();
        if (bytes_available == 0)
        {
            //The data may have just not arrived yet, so sleep a bit and check again
            //See note above on delay durcation
            delay(1);
            bytes_available = stream_->available();
        }
    }
    //If there's room in the buffer, set the next character to null for good measure
    if (buffer_pos < buffer_len)
    {
        buffer[buffer_pos] = '\0';
    }
    return buffer_pos;
}


// Close previously accepted TCP stream
// Returns true on successful unaccept, else false.
boolean SimpleESP8266::unacceptTCP()
{

    this->println(F("AT+CIPSERVER=0"));

    if (find())
    {
        //The device responds "OK" and then closes the current connections, so discard any additional input that comes
        clearStreamBuffer();
        return true;
    }
    return false;
}

void SimpleESP8266::closeTCP(void)
{
    this->println(F("AT+CIPCLOSE"));
    find(F("Unlink\r\n"));
}

// Requests page from currently-open TCP connection.  URL is
// flash-resident string.  Returns true if request issued successfully,
// else false.  Calling function should then handle data returned, may
// need to parse IPD delimiters (see notes in find() function.
boolean SimpleESP8266::requestURL(EspStr *url)
{
    this->print(F("AT+CIPSEND="));
    this->println(25 + strlen_P((Pchr *)url) + strlen_P((Pchr *)host_));

    //if (find(F("> ")))
    //{ // Wait for prompt
    this->print(F("GET ")); // 4
    this->print(url);
    this->print(F(" HTTP/1.1\r\nHost: ")); // 17
    this->print(host_);
    this->print(F("\r\n\r\n")); // 4
    return(find()); // Gets 'SEND OK' line
                    //}
    return false;
}

// Requests page from currently-open TCP connection.  URL is
// character string in SRAM.  Returns true if request issued successfully,
// else false.  Calling function should then handle data returned, may
// need to parse IPD delimiters (see notes in find() function.
boolean SimpleESP8266::requestURL(char* url)
{
    this->print(F("AT+CIPSEND="));
    //25 indicates length of total message, and is sizeof("GET HTTP/1.1\r\nHost: \r\n\r\n")
    this->println(25 + strlen(url) + strlen_P((Pchr *)host_));
    if (find(F("> ")))
    { // Wait for prompt
        this->print(F("GET ")); // 4
        this->print(url);
        this->print(F(" HTTP/1.1\r\nHost: ")); // 17
        this->print(host_);
        this->print(F("\r\n\r\n")); // 4
        return(find()); // Gets 'SEND OK' line
    }
    return false;
}

boolean SimpleESP8266::setupTcpServer(EspStr *ssid, EspStr* password, uint16_t port)
{
    // Test if module is ready
    if (debug_) debug_->println(DEBUG_STR("\r\nHard reset"));
    if (!this->hardReset())
    {
        if (debug_) debug_->println(DEBUG_STR("no response from module"));
        return false;
    }
    if (debug_) debug_->println(DEBUG_STR("OK."));

    if (debug_) debug_->print(DEBUG_STR("\r\nSoft reset"));
    if (!this->softReset())
    {
        if (debug_) debug_->println(DEBUG_STR("no response from module."));
        return false;
    }
    if (debug_) debug_->println(DEBUG_STR("OK."));

    if (debug_) debug_->print(DEBUG_STR("\r\nConnect to WiFi"));
    if (this->connectToAP(ssid, password))
    {
        char buffer[40];
        // IP addr check isn't part of library yet, but
        // we can manually request and place in a string.
        if (debug_) debug_->print(DEBUG_STR("OK\nCheck IP addr"));
        this->println(F("AT+CIFSR"));
        if (this->readLine(buffer, sizeof(buffer)))
        {
            this->find(); // Discard the 'OK' that follows

            if (debug_) debug_->print(DEBUG_STR("Accept TCP conn"));
            if (this->acceptTCP(port))
            {
                if (debug_) debug_->println(DEBUG_STR("TCP conn accepted"));
                return true;
            } else
            { // TCP connect failed
                if (debug_) debug_->println(DEBUG_STR("Fail to accept TCP conn"));
            }
        } else
        { // IP addr check failed
            if (debug_) debug_->println(DEBUG_STR("Fail to read IP addr"));
        }
    } else
    { // WiFi connection failed
        if (debug_) debug_->println(DEBUG_STR("Fail to connect to AP"));
    }
    return false;
}