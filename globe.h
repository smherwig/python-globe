#ifndef _GLOBE_H_
#define _GLOBE_H_

struct globe_dms {
    unsigned short lon_deg;
    unsigned short lon_min;
    unsigned short lon_sec;
    unsigned short east;
    unsigned short lat_deg;
    unsigned short lat_min;
    unsigned short lat_sec;
    unsigned short north;
};

/**
 * Compute the great circle distance, in meters, between two points.  The
 * points are given in degrees.
 */
double globe_gcdistance(double p0_lon, double p0_lat, double p1_lon,
        double p1_lat);

/**
 * Compute the great circle distance, in meters, between two points.  The
 * points are given in degrees.
 */
double globe_gcdistance_haversine(double p0_lon, double p0_lat, double p1_lon,
        double p1_lat);

/**
 * Compute the bearing, in degrees [0, 360) clockwise of due North, of a great
 * circle path from p0 to p1.  p0 and p1 are given in degrees.
 */
double globe_bearing(double p0_lon, double p0_lat, double p1_lon,
        double p1_lat);

/**
 * Given a starting point p0 (in degrees), initial bearing (degrees clockwise
 * of due North), and distance (meters), compute the destination point p1 (in
 * degrees).
 */
void globe_dstpoint(double p0_lon, double p0_lat, double bearing,
        double distance, double *p1_lon, double *p1_lat);

/**
 * Compute the theoretical round-trip time (in seconds) for electronic
 * communication between points p0 and p1 (in degrees).
 */
double globe_rtttheoretical(double p0_lon, double p0_lat, double p1_lon,
        double p1_lat);

/**
 * Compute the location of point i that is on the great circle path from 
 * p0 to p1, fraction of the way from p0.  For intsance, fraction=0.5 is the
 * midpoint of the great circle path.
 */
void globe_interpoint(double p0_lon, double p0_lat, double p1_lon,
        double p1_lat, double fraction, double *i_lon, double *i_lat);

void globe_degreestodms(double lon, double lat, struct globe_dms *dms);

void globe_dmstodegrees(struct globe_dms *dms, double *lon, double *lat);

/* Compute the distance from the third point (lon3, lat3) to the line 
 * segment connecting the first and second point.
 */
double globe_disttoline(double lon1, double lat1, double lon2, double lat2,
        double lon3, double lat3);


#endif /* !_GLOBE_H */
