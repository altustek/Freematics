/*************************************************************************
* Diagnostic log tee for Freematics ONE+
* With DIAG_LOG defined, FreematicsBase.h maps Serial to DiagSerial: output
* still goes to the USB serial port and is also kept, line-timestamped, in a
* RAM buffer that the application drains to storage (see DiagLog::take).
*************************************************************************/

#ifndef FREEMATICS_LOG
#define FREEMATICS_LOG

#include <Arduino.h>

#define DIAG_LOG_BUFFER_SIZE 4096

class DiagLog : public Stream
{
public:
    DiagLog();
    void begin(unsigned long baud) { ::Serial.begin(baud); }
    size_t write(uint8_t c) override;
    size_t write(const uint8_t* buf, size_t size) override;
    int available() override { return ::Serial.available(); }
    int read() override { return ::Serial.read(); }
    int peek() override { return ::Serial.peek(); }
    void flush() override { ::Serial.flush(); }
    using Print::write;
    // copy up to max buffered bytes into out, removing them; returns count
    size_t take(char* out, size_t max);
    // bytes currently buffered
    size_t pending();
    // bytes discarded because the buffer was full
    uint32_t dropped() { return m_dropped; }
    // hold back USB output (still buffered for the log), e.g. during a file
    // transfer over the same port; raw() writes to USB regardless
    void mute(bool on) { m_mute = on; }
    Print& raw() { return ::Serial; }
private:
    void append(const uint8_t* buf, size_t size);
    void put(char c);
    size_t m_head = 0;
    size_t m_tail = 0;
    uint32_t m_dropped = 0;
    bool m_lineStart = true;
    volatile bool m_mute = false;
    SemaphoreHandle_t m_lock = 0;
    bool lock();
    void unlock() { xSemaphoreGive(m_lock); }
};

extern DiagLog DiagSerial;

#endif
