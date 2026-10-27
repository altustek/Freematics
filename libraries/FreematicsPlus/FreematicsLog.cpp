/*************************************************************************
* Diagnostic log tee for Freematics ONE+ (see FreematicsLog.h)
*************************************************************************/

#include "FreematicsLog.h"

DiagLog DiagSerial;

// the buffer lives in RTC slow memory, outside the heap: with WiFi, BLE and
// HTTPD up the heap has no room to spare for the telemetry task's stack
static RTC_DATA_ATTR char m_buf[DIAG_LOG_BUFFER_SIZE];

size_t DiagLog::write(uint8_t c)
{
    return write(&c, 1);
}

size_t DiagLog::write(const uint8_t* buf, size_t size)
{
    if (!m_mute) ::Serial.write(buf, size);
    append(buf, size);
    return size;
}

DiagLog::DiagLog()
{
    m_lock = xSemaphoreCreateMutex();
}

bool DiagLog::lock()
{
    // tasks only (never from an ISR)
    return m_lock && xSemaphoreTake(m_lock, pdMS_TO_TICKS(100)) == pdTRUE;
}

void DiagLog::append(const uint8_t* buf, size_t size)
{
    // stamp formatted outside the lock; used only if a line starts here
    char stamp[16];
    uint32_t t = millis();
    int stampLen = snprintf(stamp, sizeof(stamp), "[%u.%03u] ", t / 1000, t % 1000);
    if (!lock()) {
        m_dropped += size;
        return;
    }
    for (size_t i = 0; i < size; i++) {
        if (m_lineStart) {
            for (int j = 0; j < stampLen; j++) put(stamp[j]);
            m_lineStart = false;
        }
        put(buf[i]);
        if (buf[i] == '\n') m_lineStart = true;
    }
    unlock();
}

void DiagLog::put(char c)
{
    size_t next = (m_head + 1) % DIAG_LOG_BUFFER_SIZE;
    if (next == m_tail) {
        m_dropped++;
        return;
    }
    m_buf[m_head] = c;
    m_head = next;
}

size_t DiagLog::take(char* out, size_t max)
{
    size_t n = 0;
    if (!lock()) return 0;
    while (n < max && m_tail != m_head) {
        out[n++] = m_buf[m_tail];
        m_tail = (m_tail + 1) % DIAG_LOG_BUFFER_SIZE;
    }
    unlock();
    return n;
}

size_t DiagLog::pending()
{
    if (!lock()) return 0;
    size_t n = (m_head + DIAG_LOG_BUFFER_SIZE - m_tail) % DIAG_LOG_BUFFER_SIZE;
    unlock();
    return n;
}
