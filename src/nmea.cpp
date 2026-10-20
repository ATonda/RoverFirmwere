#include "nmea.h"

#include "gnss.h"

namespace nmea {

static Sink sink = nullptr;
static uint8_t mask = GGA | RMC | GST | GSA | GSV;

void setSink(Sink s) { sink = s; }
void setMask(uint8_t m) { mask = m; }

// Dokončí větu: doplní kontrolní součet a CRLF, předá do výstupu
static void emit(char *buf, size_t cap, size_t len) {
  if (!sink || len + 6 > cap) return;
  uint8_t cs = 0;
  for (size_t i = 1; i < len; i++) cs ^= (uint8_t)buf[i];
  len += snprintf(buf + len, cap - len, "*%02X\r\n", cs);
  sink(buf, len);
}

static int fmtTime(char *b, size_t n, const gnss::Pvt &p) {
  if (!p.timeValid) return snprintf(b, n, ",");
  return snprintf(b, n, ",%02u%02u%02u.%02lu", p.hour, p.min, p.sec, (unsigned long)((p.iTOW % 1000) / 10));
}

// Souřadnice ve formátu (d)ddmm.mmmmmmm,H
static int fmtCoord(char *b, size_t n, double v, bool isLat) {
  char hemi = isLat ? (v >= 0 ? 'N' : 'S') : (v >= 0 ? 'E' : 'W');
  double a = fabs(v);
  int deg = (int)a;
  double min = (a - deg) * 60.0;
  if (min >= 59.99999995) {
    deg++;
    min = 0;
  }
  if (isLat) return snprintf(b, n, ",%02d%010.7f,%c", deg, min, hemi);
  return snprintf(b, n, ",%03d%010.7f,%c", deg, min, hemi);
}

int ggaQuality() {
  const gnss::Pvt &p = gnss::pvt();
  if (!p.valid) return 0;
  if (p.fixType == 1) return 6;  // jen dopočet
  if (!p.gnssFixOk) return 0;
  if (p.carrSoln && !gnss::rtcmActive()) return 2;  // HAS (PPP) bez RTCM = korekce, ne RTK
  if (p.carrSoln == 2) return 4;
  if (p.carrSoln == 1) return 5;
  if (p.diffSoln) return 2;
  return 1;
}

static char rmcMode() {
  switch (ggaQuality()) {
    case 1: return 'A';
    case 2: return 'D';
    case 4: return 'R';
    case 5: return 'F';
    case 6: return 'E';
    default: return 'N';
  }
}

static void sendGGA(const gnss::Pvt &p) {
  char b[128];
  int q = ggaQuality();
  int n = snprintf(b, sizeof(b), "$GNGGA");
  n += fmtTime(b + n, sizeof(b) - n, p);
  if (q == 0) {
    n += snprintf(b + n, sizeof(b) - n, ",,,,,0,%02u,,,M,,M,,", p.numSV);
  } else {
    n += fmtCoord(b + n, sizeof(b) - n, p.lat, true);
    n += fmtCoord(b + n, sizeof(b) - n, p.lon, false);
    const gnss::Dop &d = gnss::dop();
    float hdop = d.valid ? d.hDOP : p.pDOP;
    n += snprintf(b + n, sizeof(b) - n, ",%d,%02u,%.2f,%.3f,M,%.3f,M,", q, p.numSV, hdop, p.hMSL, p.hEll - p.hMSL);
    if ((q == 2 || q == 4 || q == 5) && p.corrAge) n += snprintf(b + n, sizeof(b) - n, "%.1f,", gnss::corrAgeSeconds(p.corrAge));
    else n += snprintf(b + n, sizeof(b) - n, ",");
  }
  emit(b, sizeof(b), n);
}

static void sendRMC(const gnss::Pvt &p) {
  char b[128];
  bool ok = ggaQuality() != 0;
  int n = snprintf(b, sizeof(b), "$GNRMC");
  n += fmtTime(b + n, sizeof(b) - n, p);
  n += snprintf(b + n, sizeof(b) - n, ",%c", ok ? 'A' : 'V');
  if (ok) {
    n += fmtCoord(b + n, sizeof(b) - n, p.lat, true);
    n += fmtCoord(b + n, sizeof(b) - n, p.lon, false);
    n += snprintf(b + n, sizeof(b) - n, ",%.3f,%.2f", p.gSpeed * 1.943844f, p.headMot);
  } else {
    n += snprintf(b + n, sizeof(b) - n, ",,,,,,");
  }
  if (p.timeValid) n += snprintf(b + n, sizeof(b) - n, ",%02u%02u%02u", p.day, p.month, p.year % 100);
  else n += snprintf(b + n, sizeof(b) - n, ",");
  n += snprintf(b + n, sizeof(b) - n, ",,,%c,V", rmcMode());
  emit(b, sizeof(b), n);
}

static void sendGST(const gnss::Pvt &p) {
  char b[128];
  float sN, sE, sD;
  const gnss::Cov &c = gnss::cov();
  if (c.valid && millis() - c.rxMs < 1500) {
    sN = c.sN;
    sE = c.sE;
    sD = c.sD;
  } else {
    // bez kovariance: hAcc rozdělit rovnoměrně do N a E
    sN = sE = p.hAcc / 1.41421356f;
    sD = p.vAcc;
  }
  int n = snprintf(b, sizeof(b), "$GNGST");
  n += fmtTime(b + n, sizeof(b) - n, p);
  if (ggaQuality() == 0) {  // bez fixu prázdná pole místo nesmyslných odchylek
    n += snprintf(b + n, sizeof(b) - n, ",,,,,,,");
    emit(b, sizeof(b), n);
    return;
  }
  n += snprintf(b + n, sizeof(b) - n, ",,%.3f,%.3f,0.0,%.3f,%.3f,%.3f", sN > sE ? sN : sE, sN > sE ? sE : sN, sN, sE, sD);
  emit(b, sizeof(b), n);
}

void sendEpoch() {
  const gnss::Pvt &p = gnss::pvt();
  if (!p.valid) return;
  if (mask & GGA) sendGGA(p);
  if (mask & RMC) sendRMC(p);
  if (mask & GST) sendGST(p);
}

// ---------------------------------------------------------------- GSA / GSV
struct Constellation {
  const char *talker;  // pro GSV
  uint8_t systemId;    // pro GSA (NMEA 4.10)
};

// index podle u-blox gnssId (SBAS se vykazuje pod GPS)
static bool constellationOf(uint8_t gnssId, Constellation &c) {
  switch (gnssId) {
    case 0:
    case 1: c = {"GP", 1}; return true;
    case 6: c = {"GL", 2}; return true;
    case 2: c = {"GA", 3}; return true;
    case 3: c = {"GB", 4}; return true;
    case 5: c = {"GQ", 5}; return true;
    case 7: c = {"GI", 6}; return true;
    default: return false;
  }
}

// GLONASS s neznámým slotem (svId 255) nemá platné číslo NMEA – vynechat
static bool validSv(const gnss::SatInfo &s) { return !(s.gnssId == 6 && s.svId == 255); }

static int nmeaSvId(const gnss::SatInfo &s) {
  switch (s.gnssId) {
    case 1: return s.svId - 87;  // SBAS 120–158 -> 33–71
    case 6: return s.svId + 64;  // GLONASS 1–24 -> 65–88
    default: return s.svId;
  }
}

void sendSats() {
  const gnss::SatSummary &ss = gnss::sats();
  if (!ss.valid) return;
  const gnss::Dop &d = gnss::dop();
  static const uint8_t groups[][2] = {{0, 1}, {6, 255}, {2, 255}, {3, 255}, {5, 255}, {7, 255}};

  // GSA – jedna (nebo více) vět pro každou konstelaci s použitými satelity
  for (const auto &g : groups) {
    if (!(mask & GSA)) break;
    int ids[gnss::MAX_SATS];
    int cnt = 0;
    for (int i = 0; i < ss.count; i++) {
      const gnss::SatInfo &s = ss.list[i];
      if (s.used && validSv(s) && (s.gnssId == g[0] || s.gnssId == g[1])) ids[cnt++] = nmeaSvId(s);
    }
    if (!cnt) continue;
    Constellation c;
    constellationOf(g[0], c);
    for (int start = 0; start < cnt; start += 12) {
      char b[128];
      int fix = gnss::pvt().gnssFixOk ? (gnss::pvt().fixType == 2 ? 2 : 3) : 1;
      int n = snprintf(b, sizeof(b), "$GNGSA,A,%d", fix);
      for (int k = 0; k < 12; k++) {
        if (start + k < cnt) n += snprintf(b + n, sizeof(b) - n, ",%02d", ids[start + k]);
        else n += snprintf(b + n, sizeof(b) - n, ",");
      }
      n += snprintf(b + n, sizeof(b) - n, ",%.2f,%.2f,%.2f,%u", d.pDOP, d.hDOP, d.vDOP, c.systemId);
      emit(b, sizeof(b), n);
    }
  }

  // GSV – satelity v dohledu po konstelacích, 4 na větu
  for (const auto &g : groups) {
    if (!(mask & GSV)) break;
    int idx[gnss::MAX_SATS];
    int cnt = 0;
    for (int i = 0; i < ss.count; i++) {
      const gnss::SatInfo &s = ss.list[i];
      if (validSv(s) && (s.gnssId == g[0] || s.gnssId == g[1])) idx[cnt++] = i;
    }
    if (!cnt) continue;
    Constellation c;
    constellationOf(g[0], c);
    int total = (cnt + 3) / 4;
    for (int m = 0; m < total; m++) {
      char b[128];
      int n = snprintf(b, sizeof(b), "$%sGSV,%d,%d,%02d", c.talker, total, m + 1, cnt);
      for (int k = 0; k < 4 && m * 4 + k < cnt; k++) {
        const gnss::SatInfo &s = ss.list[idx[m * 4 + k]];
        n += snprintf(b + n, sizeof(b) - n, ",%02d", nmeaSvId(s));
        if (s.elev >= -90 && s.elev <= 90) n += snprintf(b + n, sizeof(b) - n, ",%02d,%03d", s.elev, s.azim);
        else n += snprintf(b + n, sizeof(b) - n, ",,");
        if (s.cno) n += snprintf(b + n, sizeof(b) - n, ",%02u", s.cno);
        else n += snprintf(b + n, sizeof(b) - n, ",");
      }
      emit(b, sizeof(b), n);
    }
  }
}

}  // namespace nmea
