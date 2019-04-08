#include <math.h>

#include "globe.h"

#define GLOBE_PI 3.14159

/* meters */
#define GLOBE_RADIUS  6378137.0

/* meters / second */
#define GLOBE_SPEED_OF_LIGHT 299792458.0

#define GLOBE_DEGREES_TO_RADIANS(degrees) \
    (((degrees) * GLOBE_PI) / 180.0)

#define GLOBE_RADIANS_TO_DEGREES(radians) \
    ((radians) * (180.0 / (GLOBE_PI)))

#define GLOBE_ANGULAR_DISTANCE(distance) \
    ((distance) / (GLOBE_RADIUS))

double
globe_gcdistance(double p0_lon, double p0_lat, double p1_lon, double p1_lat)
{
    double p0_lon_rad = GLOBE_DEGREES_TO_RADIANS(p0_lon);
    double p0_lat_rad = GLOBE_DEGREES_TO_RADIANS(p0_lat);
    double p1_lon_rad = GLOBE_DEGREES_TO_RADIANS(p1_lon);
    double p1_lat_rad = GLOBE_DEGREES_TO_RADIANS(p1_lat);
    double sigma = 0.0;     // central angle

    sigma = sin(p0_lat_rad) * sin(p1_lat_rad) + 
        cos(p0_lat_rad) * cos(p1_lat_rad) * cos(p1_lon_rad - p0_lon_rad);

    /* return distance in meters */
    return (acos(sigma) * GLOBE_RADIUS);
}

// Compute the great circle distance between two points using the haversine formula.
double globe_gcdistance_haversine(double lon1, double lat1, double lon2, double lat2) {
	double lon1_rad = GLOBE_DEGREES_TO_RADIANS(lon1);
	double lat1_rad = GLOBE_DEGREES_TO_RADIANS(lat1);
	double lon2_rad = GLOBE_DEGREES_TO_RADIANS(lon2);
	double lat2_rad = GLOBE_DEGREES_TO_RADIANS(lat2);

	double ret = sin((lat2_rad-lat1_rad)/2)*sin((lat2_rad-lat1_rad)/2)+cos(lat1_rad)*cos(lat2_rad)*sin((lon2_rad-lon1_rad)/2)*sin((lon2_rad-lon1_rad)/2);

	return 2*atan2(sqrt(ret), sqrt(1-ret))*GLOBE_RADIUS; 
}

double
globe_bearing(double p0_lon, double p0_lat, double p1_lon, double p1_lat)
{
    double p0_lon_rad = GLOBE_DEGREES_TO_RADIANS(p0_lon);
    double p0_lat_rad = GLOBE_DEGREES_TO_RADIANS(p0_lat);
    double p1_lon_rad = GLOBE_DEGREES_TO_RADIANS(p1_lon);
    double p1_lat_rad = GLOBE_DEGREES_TO_RADIANS(p1_lat);
    double x = 0.0;
    double y = 0.0;
    double bearing = 0.0;

    x = sin(p1_lon_rad - p0_lon_rad) * cos(p1_lat_rad);
    y = cos(p0_lat_rad) * sin(p1_lat_rad) -
            sin(p0_lat_rad) * cos(p1_lat_rad) * cos(p1_lon_rad - p0_lon_rad);

    bearing = GLOBE_RADIANS_TO_DEGREES(atan2(x,y));

    /* return bearing in degrees [0, 360) clockwise of due North */
    return (fmod(bearing + 360, 360));
}

void
globe_dstpoint(double p0_lon, double p0_lat, double bearing, double distance, 
        double *p1_lon, double *p1_lat)
{
    double p0_lon_rad = GLOBE_DEGREES_TO_RADIANS(p0_lon);
    double p0_lat_rad = GLOBE_DEGREES_TO_RADIANS(p0_lat);
    double bearing_rad = GLOBE_DEGREES_TO_RADIANS(bearing);
    double p1_lon_rad = 0.0;
    double p1_lat_rad = 0.0;
    // central angle
    double sigma = GLOBE_ANGULAR_DISTANCE(distance);

    p1_lat_rad = asin(sin(p0_lat_rad) * cos(sigma) + 
            cos(p0_lat_rad) * sin(sigma) * cos(bearing_rad));

    p1_lon_rad = p0_lon_rad + 
            atan2(sin(bearing_rad) * sin(sigma) * cos(p0_lat_rad), 
                  cos(sigma) - sin(p0_lat_rad) * sin(p1_lat_rad));

    p1_lon_rad = fmod(p1_lon_rad + GLOBE_PI, 2* GLOBE_PI) - GLOBE_PI;

    *p1_lat = GLOBE_RADIANS_TO_DEGREES(p1_lat_rad);
    *p1_lon = GLOBE_RADIANS_TO_DEGREES(p1_lon_rad);
}

double
globe_rtttheoretical(double p0_lon, double p0_lat, double p1_lon, double p1_lat)
{
    double distance = 0.0;
    double rtt = 0.0;

    distance = globe_gcdistance_haversine(p0_lon, p0_lat, p1_lon, p1_lat);
    
    rtt = distance * 3 / GLOBE_SPEED_OF_LIGHT;

    return (rtt);
}

void
globe_interpoint(double p0_lon, double p0_lat, double p1_lon, double p1_lat,
        double fraction, double *i_lon, double *i_lat)
{
    double p0_lon_rad = GLOBE_DEGREES_TO_RADIANS(p0_lon);
    double p0_lat_rad = GLOBE_DEGREES_TO_RADIANS(p0_lat);
    double p1_lon_rad = GLOBE_DEGREES_TO_RADIANS(p1_lon);
    double p1_lat_rad = GLOBE_DEGREES_TO_RADIANS(p1_lat);
    double i_lon_rad = 0.0;
    double i_lat_rad = 0.0;
    double a = 0.0;
    double b = 0.0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double distance = 0.0;
    double sigma = 0.0; 

    distance = globe_gcdistance_haversine(p0_lon, p0_lat, p1_lon, p1_lat);
    sigma = GLOBE_ANGULAR_DISTANCE(distance);

    a = sin((1-fraction) * sigma) / sin(sigma);
    b = sin(fraction * sigma) / sin(sigma);

    x = (a * cos(p0_lat_rad) * cos(p0_lon_rad)) + 
        (b * cos(p1_lat_rad) * cos(p1_lon_rad));

    y = (a * cos(p0_lat_rad) * sin(p0_lon_rad)) + 
        (b * cos(p1_lat_rad) * sin(p1_lon_rad));

    z = a * sin(p0_lat_rad) + b * sin(p1_lat_rad);

    i_lat_rad = atan2(z, sqrt(x*x + y*y));
    i_lon_rad = atan2(y, x);

    *i_lat = GLOBE_RADIANS_TO_DEGREES(i_lat_rad);
    *i_lon = GLOBE_RADIANS_TO_DEGREES(i_lon_rad);
}

void
globe_degreestodms(double lon, double lat, struct globe_dms *dms)
{
    double lon_abs = fabs(lon);
    double lat_abs = fabs(lat);

    dms->lon_deg = (unsigned int)(floor(lon_abs));
    dms->lon_min = (unsigned int)(fmod(floor(lon_abs * 60), 60));
    dms->lon_sec = (unsigned int)fmod(lon_abs * 3600, 60);
    dms->east = lon < 0 ? 0 : 1;

    dms->lat_deg = (unsigned int)(floor(lat_abs));
    dms->lat_min = (unsigned int)(fmod(floor(lat_abs * 60), 60));
    dms->lat_sec = (unsigned int)fmod(lat_abs * 3600, 60);
    dms->north = lat < 0 ? 0 : 1;
}

void
globe_dmstodegrees(struct globe_dms *dms, double *lon, double *lat)
{
    *lon = dms->lon_deg + dms->lon_min / 60.0 + dms->lon_sec / 3600.0;
    if (!dms->east)
        *lon *= -1;

    *lat = dms->lat_deg + dms->lat_min / 60.0 + dms->lat_sec / 3600.0;
    if (!dms->north)
        *lon *= -1;
}

/* Compute the great circle distance between two points using the spherical
 * law of cosines.
 */
static double
dist_cosine(double lon1, double lat1, double lon2, double lat2)
{
    double lon1_rad = GLOBE_DEGREES_TO_RADIANS(lon1);
    double lat1_rad = GLOBE_DEGREES_TO_RADIANS(lat1);
    double lon2_rad = GLOBE_DEGREES_TO_RADIANS(lon2);
    double lat2_rad = GLOBE_DEGREES_TO_RADIANS(lat2);

    return acos(sin(lat1_rad)*sin(lat2_rad) + cos(lat1_rad)*cos(lat2_rad)*cos(lon2_rad-lon1_rad))*GLOBE_RADIUS;
}

// Compute the initial beraing of the line connecting given two points.
static double
bearing(double lon1, double lat1, double lon2, double lat2)
{
    double lon1_rad = GLOBE_DEGREES_TO_RADIANS(lon1);
    double lat1_rad = GLOBE_DEGREES_TO_RADIANS(lat1);
    double lon2_rad = GLOBE_DEGREES_TO_RADIANS(lon2);
    double lat2_rad = GLOBE_DEGREES_TO_RADIANS(lat2);

    double ret = atan2(sin(lon2_rad-lon1_rad)*cos(lat2_rad), cos(lat1_rad)*sin(lat2_rad)-sin(lat1_rad)*cos(lat2_rad)*cos(lon2_rad-lon1_rad)) * 180.0/GLOBE_PI;
	
	return fmod(ret+360, 360);
}

/* Compute the great circle distance between two points using the haversine
 * formula.
 */
static double 
dist_haversine(double lon1, double lat1, double lon2, double lat2) 
{
    double lon1_rad = GLOBE_DEGREES_TO_RADIANS(lon1);
    double lat1_rad = GLOBE_DEGREES_TO_RADIANS(lat1);
    double lon2_rad = GLOBE_DEGREES_TO_RADIANS(lon2);
    double lat2_rad = GLOBE_DEGREES_TO_RADIANS(lat2);

    double ret = sin((lat2_rad-lat1_rad)/2)*sin((lat2_rad-lat1_rad)/2)+cos(lat1_rad)*cos(lat2_rad)*sin((lon2_rad-lon1_rad)/2)*sin((lon2_rad-lon1_rad)/2);

    return 2*atan2(sqrt(ret), sqrt(1-ret))*GLOBE_RADIUS; 
}

/* Compute the cross track distance.  */
static double
dist_to_gc(double lon1, double lat1, double lon2, double lat2, double lon3, double lat3)
{
    double b12 = GLOBE_DEGREES_TO_RADIANS(bearing(lon1, lat1, lon2, lat2));
    double b13 = GLOBE_DEGREES_TO_RADIANS(bearing(lon1, lat1, lon3, lat3));
    double d13 = dist_cosine(lon1, lat1, lon3, lat3)/GLOBE_RADIUS;

	return	asin(sin(d13)*sin(b13-b12))*GLOBE_RADIUS;
}

/* Compute the along track distance.  */
static double
along_track_distance(double lon1, double lat1, double lon2, double lat2, 
        double lon3, double lat3) 
{
    double dxt = dist_to_gc(lon1, lat1, lon2, lat2, lon3, lat3)/GLOBE_RADIUS;
    double d13 = dist_cosine(lon1, lat1, lon3, lat3)/GLOBE_RADIUS;
    return acos(cos(d13)/cos(dxt))*GLOBE_RADIUS;
}

/* Compute the distance from the third point (lon3, lat3) to the line 
 * segment connecting the first and second point.
 */
double 
globe_disttoline(double lon1, double lat1, double lon2, double lat2,
        double lon3, double lat3) 
{
    double seg_length = dist_haversine(lon1, lat1, lon2, lat2);

    double dat1 = along_track_distance(lon1, lat1, lon2, lat2, lon3, lat3);
    double dat2 = along_track_distance(lon2, lat2, lon1, lat1, lon3, lat3);

    double min_dat, max_dat;
    if(dat1 < dat2) {
        min_dat = dat1;
        max_dat = dat2;
    }
    else {
        max_dat = dat1;
        min_dat = dat2;
    }

    double d13 = dist_haversine(lon1, lat1, lon3, lat3);
    double d23 = dist_haversine(lon2, lat2, lon3, lat3);
    double min_node_dist;

    if(d13 < d23)
        min_node_dist = d13;
    else
        min_node_dist = d23;

    if(max_dat >= seg_length) {
        return min_node_dist;
    }
    else{
        double dxt = fabs(dist_to_gc(lon1, lat1, lon2, lat2, lon3, lat3));
        if(dxt < min_node_dist)
            return dxt;
        else
            return min_node_dist;
    }
}

