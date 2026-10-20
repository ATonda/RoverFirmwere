#include "geo.h"

#include <math.h>

namespace geo {

static constexpr double A = 6378137.0;
static constexpr double F = 1.0 / 298.257222101;
static constexpr double E2 = F * (2 - F);
static constexpr double DEG = M_PI / 180.0;
static constexpr double MAS = M_PI / 180.0 / 3600000.0;

Ecef toEcef(double latDeg, double lonDeg, double h) {
  double la = latDeg * DEG, lo = lonDeg * DEG;
  double s = sin(la);
  double N = A / sqrt(1 - E2 * s * s);
  return {(N + h) * cos(la) * cos(lo), (N + h) * cos(la) * sin(lo), (N * (1 - E2) + h) * s};
}

void fromEcef(const Ecef &p, double &latDeg, double &lonDeg, double &h) {
  double r = sqrt(p.x * p.x + p.y * p.y);
  double la = atan2(p.z, r * (1 - E2));
  h = 0;
  for (int i = 0; i < 6; i++) {
    double s = sin(la);
    double N = A / sqrt(1 - E2 * s * s);
    h = r / cos(la) - N;
    la = atan2(p.z, r * (1 - E2 * N / (N + h)));
  }
  latDeg = la / DEG;
  lonDeg = atan2(p.y, p.x) / DEG;
}

void enu(const Ecef &a, const Ecef &b, double latDeg, double lonDeg, double &e, double &n, double &u) {
  double dx = b.x - a.x, dy = b.y - a.y, dz = b.z - a.z;
  double la = latDeg * DEG, lo = lonDeg * DEG;
  double sla = sin(la), cla = cos(la), slo = sin(lo), clo = cos(lo);
  e = -slo * dx + clo * dy;
  n = -sla * clo * dx - sla * slo * dy + cla * dz;
  u = cla * clo * dx + cla * slo * dy + sla * dz;
}

Ecef itrfToEtrs(const Ecef &p, double epochYear) {
  double dt = epochYear - 1989.0;
  double r1 = 0.086 * MAS * dt, r2 = 0.519 * MAS * dt, r3 = -0.753 * MAS * dt;
  return {p.x - r3 * p.y + r2 * p.z, p.y + r3 * p.x - r1 * p.z, p.z - r2 * p.x + r1 * p.y};
}

double epochYear(int year, int month, int day) {
  static const int cum[] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};
  int doy = cum[(month >= 1 && month <= 12) ? month - 1 : 0] + day;
  bool leap = (year % 4 == 0 && year % 100 != 0) || year % 400 == 0;
  if (leap && month > 2) doy++;
  return year + (doy - 0.5) / (leap ? 366.0 : 365.0);
}

}  // namespace geo
