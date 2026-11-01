// Host test for firmware_v5/telelogger/gnss_gate.h
//   build: c++ -std=c++11 -O1 replay.cpp -o replay
//   run:   ./replay selftest
//          ./replay replay <DATA/n.CSV>...     (telelogger SD CSV files)
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <string>
#include <fstream>
#include <vector>
#include "../../firmware_v5/telelogger/gnss_gate.h"

struct Sample { double lat, lng, alt, kph, obd; uint32_t ms; bool hasObd; };

// CSV: one "key,value" per line; a sample ends with key 82 (device temperature, added last)
static std::vector<Sample> load(const char* path)
{
  std::ifstream f(path);
  std::vector<Sample> out;
  std::string line;
  bool hasA = false, hasB = false;
  double lat = 0, lng = 0, alt = 0, kph = 0, obd = -1;
  double gpst = -1;
  double base = -1, prev = -1, wrap = 0;
  while (std::getline(f, line)) {
    size_t c = line.find(',');
    if (c == std::string::npos) continue;
    std::string k = line.substr(0, c);
    const char* v = line.c_str() + c + 1;
    if (k == "A") { lat = atof(v); hasA = true; }
    else if (k == "B") { lng = atof(v); hasB = true; }
    else if (k == "C") alt = atof(v);
    else if (k == "D") kph = atof(v);
    else if (k == "10D") obd = atof(v);
    else if (k == "10") {
      long t = atol(v); // hhmmsscc
      gpst = (t / 1000000) * 3600.0 + ((t / 10000) % 100) * 60.0 + ((t / 100) % 100) + (t % 100) / 100.0;
    } else if (k == "82") {
      if (hasA && hasB && gpst >= 0) {
        if (base < 0) base = gpst;
        if (prev >= 0 && gpst + wrap < prev) wrap += 86400;
        prev = gpst + wrap;
        Sample s;
        s.lat = lat; s.lng = lng; s.alt = alt; s.kph = kph;
        s.obd = obd; s.hasObd = obd >= 0;
        s.ms = (uint32_t)((gpst + wrap - base) * 1000.0);
        out.push_back(s);
      }
      hasA = hasB = false; obd = -1; gpst = -1;
    }
  }
  return out;
}

static int replay(const char* path)
{
  std::vector<Sample> v = load(path);
  GnssGate g;
  int acc = 0, rej[8] = {0};
  int first = -1;
  double maxFromFirst = 0;
  float aLat = 0, aLng = 0;
  for (size_t i = 0; i < v.size(); i++) {
    const Sample& s = v[i];
    bool ok = g.accept((float)s.lat, (float)s.lng, (float)s.alt, (float)s.kph, s.hasObd ? (float)s.obd : -1.0f, s.ms);
    if (ok) {
      if (first < 0) { first = (int)i; aLat = (float)s.lat; aLng = (float)s.lng; }
      acc++;
      double d = GnssGate::distanceM(aLat, aLng, (float)s.lat, (float)s.lng);
      if (d > maxFromFirst) maxFromFirst = d;
    } else {
      rej[g.reason()]++;
      if (getenv("GATE_VERBOSE") && g.reason() != GnssGate::WARMUP)
        printf("  #%zu %s lat=%.5f lng=%.5f alt=%.0f gps=%.0f obd=%.0f t=%.1fs\n", i, GnssGate::reasonText(g.reason()), s.lat, s.lng, s.alt, s.kph, s.hasObd ? s.obd : -1.0, s.ms / 1000.0);
    }
  }
  printf("%-40s samples=%-5zu accepted=%-5d rejected: warmup=%d speed=%d alt=%d obd=%d jump=%d value=%d  first-accepted=#%d  span=%.1f km\n",
         path, v.size(), acc, rej[GnssGate::WARMUP], rej[GnssGate::SPEED], rej[GnssGate::ALTITUDE],
         rej[GnssGate::OBD_MISMATCH], rej[GnssGate::JUMP], rej[GnssGate::BAD_VALUE], first, maxFromFirst / 1000.0);
  return 0;
}

static int fails = 0;
#define CHECK(c) do { if (!(c)) { printf("FAIL line %d: %s\n", __LINE__, #c); fails++; } } while (0)

static int selftest()
{
  const float lat0 = 42.12f, lng0 = -71.18f;
  { // stationary good fixes: 3 needed, then every one accepted
    GnssGate g;
    CHECK(!g.accept(lat0, lng0, 80, 0, -1, 0));
    CHECK(!g.accept(lat0, lng0, 80, 0, -1, 1000));
    CHECK(g.accept(lat0, lng0, 80, 0, -1, 2000));
    CHECK(g.accept(lat0, lng0, 80, 0, -1, 3000));
  }
  { // smooth wrong track at 387 km/h (the 2026-10-30 case) never locks
    GnssGate g;
    int acc = 0;
    for (int i = 0; i < 100; i++) acc += g.accept(43.08f - i * 0.0008f, -71.82f + i * 0.0003f, 1800, 387, 79, i * 1000);
    CHECK(acc == 0);
  }
  { // plausible speed but altitude garbage never locks
    GnssGate g;
    int acc = 0;
    for (int i = 0; i < 20; i++) acc += g.accept(lat0 + i * 0.0001f, lng0, -1097, 40, 40, i * 1000);
    CHECK(acc == 0);
  }
  { // GPS says 90 km/h while the car (OBD) says 0 -> rejected
    GnssGate g;
    CHECK(!g.accept(lat0, lng0, 80, 90, 0, 0));
    CHECK(g.reason() == GnssGate::OBD_MISMATCH);
  }
  { // normal drive north at 60 km/h, 1 Hz: accepted after warmup
    GnssGate g;
    int acc = 0;
    for (int i = 0; i < 60; i++) acc += g.accept(lat0 + i * 16.7f / 111195.0f, lng0, 80, 60, 60, i * 1000);
    CHECK(acc == 58);
  }
  { // one 479 m jump in a good drive: that sample is rejected, the drive carries on
    GnssGate g;
    int acc = 0, jumped = 0;
    for (int i = 0; i < 40; i++) {
      float lat = lat0 + i * 13.9f / 111195.0f;
      if (i == 20) { lat += 479.0f / 111195.0f; jumped = !g.accept(lat, lng0, 80, 50, 50, i * 1000); }
      else acc += g.accept(lat, lng0, 80, 50, 50, i * 1000);
    }
    CHECK(jumped);
    CHECK(acc == 37); // 40 - 2 warmup - 1 jump
  }
  { // receiver restarts into a different place (3 bad in a row): must re-lock
    GnssGate g;
    for (int i = 0; i < 10; i++) g.accept(lat0, lng0, 80, 0, -1, i * 1000);
    CHECK(g.locked());
    g.accept(lat0 + 0.5f, lng0, 80, 0, -1, 11000);
    g.accept(lat0 + 0.5f, lng0, 80, 0, -1, 12000);
    g.accept(lat0 + 0.5f, lng0, 80, 0, -1, 13000);
    CHECK(!g.locked());
    CHECK(!g.accept(lat0 + 0.5f, lng0, 80, 0, -1, 14000));
    CHECK(!g.accept(lat0 + 0.5f, lng0, 80, 0, -1, 15000));
    CHECK(g.accept(lat0 + 0.5f, lng0, 80, 0, -1, 16000));
  }
  { // millis() wrap
    GnssGate g;
    uint32_t t = 0xFFFFFFFFu - 1500;
    CHECK(!g.accept(lat0, lng0, 80, 0, -1, t));
    CHECK(!g.accept(lat0, lng0, 80, 0, -1, t + 1000));
    CHECK(g.accept(lat0, lng0, 80, 0, -1, t + 2000));
  }
  printf(fails ? "selftest: %d FAILED\n" : "selftest: all passed\n", fails);
  return fails ? 1 : 0;
}

int main(int argc, char** argv)
{
  if (argc >= 2 && !strcmp(argv[1], "selftest")) return selftest();
  if (argc >= 3 && !strcmp(argv[1], "replay")) { for (int i = 2; i < argc; i++) replay(argv[i]); return 0; }
  fprintf(stderr, "usage: replay selftest | replay file.CSV...\n");
  return 2;
}
