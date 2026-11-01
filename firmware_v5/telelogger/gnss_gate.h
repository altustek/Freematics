// Plausibility gate for GNSS positions.
//
// A freshly started receiver can output a smooth but wrong track (3-6 satellites, hundreds of
// km/h, altitudes of +-2 km). Nothing downstream rejects it, so such positions reach Traccar. This
// gate lets a position through only when it is physically plausible and consistent with the
// samples before it. No Arduino dependencies: the same header is replayed on the host against
// recorded trips (tests/gnss_gate/).

#pragma once
#include <math.h>
#include <stdint.h>

#ifndef GNSS_GATE_MAX_KPH
#define GNSS_GATE_MAX_KPH 250.0f
#endif
#ifndef GNSS_GATE_MIN_ALT
#define GNSS_GATE_MIN_ALT -200.0f
#endif
#ifndef GNSS_GATE_MAX_ALT
#define GNSS_GATE_MAX_ALT 3000.0f
#endif
// GPS and OBD speed may differ by this much (both above GNSS_GATE_OBD_MIN_KPH to be compared)
#ifndef GNSS_GATE_OBD_TOLERANCE_KPH
#define GNSS_GATE_OBD_TOLERANCE_KPH 30.0f
#endif
#ifndef GNSS_GATE_OBD_MIN_KPH
#define GNSS_GATE_OBD_MIN_KPH 20.0f
#endif
// distance between two accepted fixes <= speed * time * factor + margin
#ifndef GNSS_GATE_MARGIN_M
#define GNSS_GATE_MARGIN_M 50.0f
#endif
#ifndef GNSS_GATE_SPEED_FACTOR
#define GNSS_GATE_SPEED_FACTOR 1.5f
#endif
// consecutive consistent fixes needed before the first position is let through
#ifndef GNSS_GATE_LOCK_COUNT
#define GNSS_GATE_LOCK_COUNT 3
#endif
// consecutive rejections after which a locked gate has to lock again
#ifndef GNSS_GATE_RELOCK_BAD
#define GNSS_GATE_RELOCK_BAD 3
#endif

class GnssGate {
public:
  enum Reason { OK = 0, WARMUP, SPEED, ALTITUDE, OBD_MISMATCH, JUMP, BAD_VALUE };

  GnssGate() { reset(); }

  void reset()
  {
    m_locked = false;
    m_haveRef = false;
    m_chain = 0;
    m_bad = 0;
    m_reason = OK;
  }

  bool locked() const { return m_locked; }
  Reason reason() const { return m_reason; }
  static const char* reasonText(Reason r)
  {
    static const char* t[] = {"ok", "warmup", "speed", "altitude", "obd", "jump", "value"};
    return t[r];
  }

  // lat/lng in degrees, alt in metres, kph = GPS speed, obdKph < 0 if unknown/stale,
  // nowMs = any monotonic millisecond clock (wraps are fine)
  bool accept(float lat, float lng, float alt, float kph, float obdKph, uint32_t nowMs)
  {
    Reason r = plausible(lat, lng, alt, kph, obdKph);
    bool cont = true;
    if (r == OK && m_haveRef) {
      cont = continuous(lat, lng, kph, nowMs);
      if (!cont) r = JUMP;
    }

    if (m_locked) {
      if (r == OK) {
        commit(lat, lng, kph, nowMs);
        m_bad = 0;
        m_reason = OK;
        return true;
      }
      m_reason = r;
      if (++m_bad >= GNSS_GATE_RELOCK_BAD) {
        m_locked = false;
        m_haveRef = false;
        m_chain = 0;
        m_bad = 0;
      }
      return false;
    }

    // not locked: need GNSS_GATE_LOCK_COUNT plausible fixes that follow each other
    if (r != OK && r != JUMP) {
      m_chain = 0;
      m_haveRef = false;
      m_reason = r;
      return false;
    }
    if (r == JUMP) m_chain = 0; // start a new chain from this sample
    commit(lat, lng, kph, nowMs);
    if (++m_chain >= GNSS_GATE_LOCK_COUNT) {
      m_locked = true;
      m_bad = 0;
      m_reason = OK;
      return true;
    }
    m_reason = WARMUP;
    return false;
  }

private:
  bool m_locked, m_haveRef;
  uint8_t m_chain, m_bad;
  Reason m_reason;
  float m_lat, m_lng, m_kph;
  uint32_t m_ms;

  static Reason plausible(float lat, float lng, float alt, float kph, float obdKph)
  {
    if (!(lat >= -90 && lat <= 90 && lng >= -180 && lng <= 180) || (lat == 0 && lng == 0)) return BAD_VALUE;
    if (!(kph >= 0 && kph <= GNSS_GATE_MAX_KPH)) return SPEED;
    if (!(alt >= GNSS_GATE_MIN_ALT && alt <= GNSS_GATE_MAX_ALT)) return ALTITUDE;
    if (obdKph >= 0 && (kph > GNSS_GATE_OBD_MIN_KPH || obdKph > GNSS_GATE_OBD_MIN_KPH) &&
        fabsf(kph - obdKph) > GNSS_GATE_OBD_TOLERANCE_KPH) return OBD_MISMATCH;
    return OK;
  }

  bool continuous(float lat, float lng, float kph, uint32_t nowMs) const
  {
    float dt = (uint32_t)(nowMs - m_ms) / 1000.0f;
    float fast = kph > m_kph ? kph : m_kph;
    return distanceM(m_lat, m_lng, lat, lng) <= fast / 3.6f * dt * GNSS_GATE_SPEED_FACTOR + GNSS_GATE_MARGIN_M;
  }

  void commit(float lat, float lng, float kph, uint32_t nowMs)
  {
    m_lat = lat;
    m_lng = lng;
    m_kph = kph;
    m_ms = nowMs;
    m_haveRef = true;
  }

public:
  // flat-earth distance, accurate enough for fixes seconds apart
  static float distanceM(float lat1, float lng1, float lat2, float lng2)
  {
    const float k = 111195.0f; // metres per degree of latitude
    float dy = (lat2 - lat1) * k;
    float dx = (lng2 - lng1) * k * cosf((lat1 + lat2) * 0.5f * 0.01745329f);
    return sqrtf(dx * dx + dy * dy);
  }
};
