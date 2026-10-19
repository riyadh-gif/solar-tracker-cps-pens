// Driver SolTrack: baca "Y M D h m s" (UT) dari stdin, tulis "az alt altRefract" (derajat, utara=0, CW).
// Azimut SolTrack hanya satu (tanpa koreksi refraksi). Argumen: lintang bujur (derajat, bujur positif timur).
// Build: gcc -O2 -I/tmp/SolTrack soltrack_driver.c /tmp/SolTrack/SolTrack.c -lm -o soltrack_driver
#include <stdio.h>
#include <stdlib.h>
#include "SolTrack.h"

int main(int argc, char **argv) {
  if (argc < 3) return 1;
  struct Location loc = {0};
  loc.latitude = atof(argv[1]); loc.longitude = atof(argv[2]);  // derajat (useDegrees=1; sinLat/cosLat dihitung internal)
  loc.pressure = 101.325; loc.temperature = 285.15;  // kPa, K (sama dengan pvlib: 101325 Pa, 12 C)
  struct Time t; struct Position p;
  while (scanf("%d %d %d %d %d %lf", &t.year, &t.month, &t.day, &t.hour, &t.minute, &t.second) == 6) {
    SolTrack(t, loc, &p, 1, 1, 0, 0);  // derajat, utara=0, tanpa refraksi ekuatorial/jarak
    printf("%.9f %.9f %.9f\n", p.azimuthRefract, p.altitude, p.altitudeRefract);
  }
  return 0;
}
