// Geodetické výpočty: ECEF, lokální ENU, přepočet ITRF2020 -> ETRS89 (ETRF2020)
#pragma once

namespace geo {

struct Ecef {
  double x, y, z;
};

// GRS80 (ETRS89) a WGS84 se liší zanedbatelně (< 0,1 mm) – použit GRS80
Ecef toEcef(double latDeg, double lonDeg, double h);

// ECEF -> zeměpisné souřadnice (GRS80)
void fromEcef(const Ecef &p, double &latDeg, double &lonDeg, double &h);

// Rozdíl (b - a) v lokálních souřadnicích u bodu (latDeg, lonDeg) [m]
void enu(const Ecef &a, const Ecef &b, double latDeg, double lonDeg, double &e, double &n, double &u);

// ITRF2020 -> ETRF2020 (EUREF): jen rotace Eurasijské desky od epochy 1989.0,
// rychlosti 0,086 / 0,519 / −0,753 mas/rok. V ČR ~0,9 m k JZ (2026).
// ČÚZK používá ETRF2000 – rozdíl proti ETRF2020 je řádu cm.
Ecef itrfToEtrs(const Ecef &p, double epochYear);

// Epocha jako desetinný rok z data
double epochYear(int year, int month, int day);

}  // namespace geo
