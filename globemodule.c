#include <Python.h>

#include "globe.h"

static PyObject *
globemodule_gcdistance(PyObject *self, PyObject *args)
{
    double p0_lon = 0.0;
    double p0_lat = 0.0;
    double p1_lon = 0.0;
    double p1_lat = 0.0;
    double distance = 0.0;

    if (!PyArg_ParseTuple(args, "dddd", &p0_lon, &p0_lat, &p1_lon, &p1_lat))
        return (NULL);

    distance = globe_gcdistance(p0_lon, p0_lat, p1_lon, p1_lat);

    return (Py_BuildValue("d", distance)); 
}

PyDoc_STRVAR(globemodule_gcdistance_doc,
"gcdistance(p0_lon, p0_lat, p1_lon, p1_lat) -> distance\n\
\n\
Given the longitude and latitude (in degrees) of two points, p0 and p1,\n\
compute the great-circle distance (in meters) between p0 and p1 using the\n\
spherical law of cosines.");

static PyObject *
globemodule_gcdistance_haversine(PyObject *self, PyObject *args)
{
    double p0_lon = 0.0;
    double p0_lat = 0.0;
    double p1_lon = 0.0;
    double p1_lat = 0.0;
    double distance = 0.0;

    if (!PyArg_ParseTuple(args, "dddd", &p0_lon, &p0_lat, &p1_lon, &p1_lat))
        return (NULL);

    distance = globe_gcdistance_haversine(p0_lon, p0_lat, p1_lon, p1_lat);

    return (Py_BuildValue("d", distance)); 
}

PyDoc_STRVAR(globemodule_gcdistance_haversine_doc,
"gcdistance_haversince(p0_lon, p0_lat, p1_lon, p1_lat) -> distance\n\
\n\
Given the longitude and latitude (in degrees) of two points, p0 and p1,\n\
compute the great-circle distance between p0 and p1 (in meters) using the\n\
Haversine formula.");

static PyObject *
globemodule_bearing(PyObject *self, PyObject *args)
{
    double p0_lon = 0.0;
    double p0_lat = 0.0;
    double p1_lon = 0.0;
    double p1_lat = 0.0;
    double bearing = 0.0;

    if (!PyArg_ParseTuple(args, "dddd", &p0_lon, &p0_lat, &p1_lon, &p1_lat))
        return (NULL);

    bearing = globe_bearing(p0_lon, p0_lat, p1_lon, p1_lat);

    return (Py_BuildValue("d", bearing)); 
}

PyDoc_STRVAR(globemodule_bearing_doc,
"bearing(p0_lon, p0_lat, p1_lon, p1_lat) -> degrees\n\
\n\
Given the longitude and latitude (in degrees) of two points, p0 and p1,\n\
compute the initial bearing (degrees clockwise of North) of the\n\
great-circle path from p0 to p1.");

static PyObject *
globemodule_dstpoint(PyObject *self, PyObject *args)
{
    double p0_lon = 0.0;
    double p0_lat = 0.0;
    double bearing = 0.0;
    double distance = 0.0;
    double p1_lon = 0.0;
    double p1_lat = 0.0;

    if (!PyArg_ParseTuple(args, "dddd", &p0_lon, &p0_lat, &bearing, &distance))
        return (NULL);

    globe_dstpoint(p0_lon, p0_lat, bearing, distance, &p1_lon, &p1_lat);

    return (Py_BuildValue("dd", p1_lon, p1_lat)); 
}

PyDoc_STRVAR(globemodule_dstpoint_doc,
"dstpoint(p0_lon, p0_lat, bearing, distance) -> (p1_lon, p1_lat)\n\
\n\
Given an initial longitude and latitude, p0_lon and p0_lat, compute the\n\
longitude and latitude, p1_lon and p1_lat, of the point that is reached by\n\
traveling a given distance with an initial bearing.\n\
\n\
Longitude and latitude are in degrees; bearing in degrees clockwise of North;\n\
distance in meters.");

static PyObject *
globemodule_rtttheoretical(PyObject *self, PyObject *args)
{
    double p0_lon = 0.0;
    double p0_lat = 0.0;
    double p1_lon = 0.0;
    double p1_lat = 0.0;
    double rtt = 0.0;

    if (!PyArg_ParseTuple(args, "dddd", &p0_lon, &p0_lat, &p1_lon, &p1_lat))
        return (NULL);

    rtt = globe_rtttheoretical(p0_lon, p0_lat, p1_lon, p1_lat);

    /* in seconds */
    return (Py_BuildValue("d", rtt)); 
}

PyDoc_STRVAR(globemodule_rtttheoretical_doc,
"rtttheoretical(p0_lon, p0_lat, p1_lon, p1_lat) -> secs\n\
\n\
Compute the theoretical minimum time in seconds for data to travel\n\
from p0 to p1 and back.  We use the observation that, in practice,\n\
data does not travel faster than (2/3) C, where C is the speed-of-light.\n\
Thus, the formula used is: secs =  3 * distance(p0,p1) / C.");

static PyObject *
globemodule_interpoint(PyObject *self, PyObject *args)
{
    double p0_lon = 0.0;
    double p0_lat = 0.0;
    double p1_lon = 0.0;
    double p1_lat = 0.0;
    double fraction = 0.0;
    double i_lon = 0.0;
    double i_lat = 0.0;

    if (!PyArg_ParseTuple(args, "ddddd", &p0_lon, &p0_lat, &p1_lon, &p1_lat, 
                &fraction))
        return (NULL);

    globe_interpoint(p0_lon, p0_lat, p1_lon, p1_lat, fraction, &i_lon, &i_lat);

    return (Py_BuildValue("dd", i_lon, i_lat)); 
}

PyDoc_STRVAR(globemodule_interpoint_doc,
"interpoint(p0_lon, p1_lat, p0_lon, p1_lat, frac) -> (px_lon, px_lat)\n\
\n\
Compute a point a fraction of the distance on the great-circle path between two points.\n\
For instance, to compute the midpoint, set frac to 0.5.\n\
\n\
Latitude and longitude are given in degrees.");

static PyObject *
globemodule_degreestodms(PyObject *self, PyObject *args)
{
    double lon = 0.0;
    double lat = 0.0;
    struct globe_dms dms = { 0 };

    if (!PyArg_ParseTuple(args, "dd", &lon, &lat))
        return (NULL);

    globe_degreestodms(lon, lat, &dms);

    return (Py_BuildValue("((HHHc),(HHHc))",
                dms.lon_deg, dms.lon_min, dms.lon_sec, dms.east ? 'E' : 'W',
                dms.lat_deg, dms.lat_min, dms.lat_sec, dms.north ? 'N' : 'S'));
}

PyDoc_STRVAR(globemodule_degreestodms_doc,
"degreestodms(lon, lat) ->\n\
    ((lon_deg, lon_min, lon_sec, 'E'|'W'), (lat_deg, lat_min, lat_sec 'N'|'S'))\n\
\n\
Convert decimal degrees to DMS (Degrees, Minute, Seconds) coordinates.\n\
\n\
For instance, to New York City has longitude=-70.0059 and latitde=40.7128,\n\
degreestodms(-70.0059, 40.7128) -> ((74, 0, 21, 'W'), (40, 42, 46, 'N'))");

static PyObject *
globemodule_dmstodegrees(PyObject *self, PyObject *args)
{
    struct globe_dms dms = { 0 };
    char eastwest = '\0';
    char northsouth = '\0';
    double lon = 0.0;
    double lat = 0.0;

    if (!PyArg_ParseTuple(args, "(HHHc)(HHHc)",
            &dms.lon_deg, &dms.lon_min, &dms.lon_sec, &eastwest,
            &dms.lat_deg, &dms.lat_min, &dms.lat_sec, &northsouth))
        return (NULL);

    dms.east = eastwest == 'E' ? 1 : 0;
    dms.north = northsouth == 'N' ? 1 : 0;

    globe_dmstodegrees(&dms, &lon, &lat);

    return (Py_BuildValue("dd", lon, lat));
}

PyDoc_STRVAR(globemodule_dmstodegrees_doc,
"dmstodegrees((lon_deg, lon_min, lon_sec, 'E'|'W'), (lat_deg, lat_min, lat_sec, 'N'|'S') -> (lon, lat)\n\
\n\
Convert DMS (Degrees, Minute, Seconds) coordinates to decimal degree coordinates.\n\
For instance, using the coordinates for New York City as an example, we have\n\
dmstodegrees((74, 0, 21, 'W'), (40, 42, 46, 'N')) -> (-74.0058, 40.7128)");

static PyObject *
globemodule_disttoline(PyObject *self, PyObject *args)
{
    double p0_lon = 0.0;
    double p0_lat = 0.0;
    double p1_lon = 0.0;
    double p1_lat = 0.0;
    double i_lon = 0.0;
    double i_lat = 0.0;
    double dist = 0.0;

    if (!PyArg_ParseTuple(args, "dddddd", &p0_lon, &p0_lat, &p1_lon, &p1_lat, 
                &i_lon, &i_lat))
        return (NULL);

    dist = globe_disttoline(p0_lon, p0_lat, p1_lon, p1_lat, i_lon, i_lat);

    return (Py_BuildValue("d", dist));
}

PyDoc_STRVAR(globemodule_disttoline_doc,
"disttoline(p0_lon, p0_lat, p1_lon, p1_lat, i_lon, i_lat) -> meters\n\
\n\
Compute the distance (in meters) from the point i to the great-circle\n\
arc connecting points p0 and p1.");

static PyMethodDef globemodule_methods[] = {
    {"gcdistance", globemodule_gcdistance, METH_VARARGS,
        globemodule_gcdistance_doc},
    {"gcdistance_haversine", globemodule_gcdistance_haversine, METH_VARARGS, 
        globemodule_gcdistance_haversine_doc},
    {"bearing", globemodule_bearing, METH_VARARGS, 
        globemodule_bearing_doc},
    {"dstpoint", globemodule_dstpoint, METH_VARARGS, 
        globemodule_dstpoint_doc},
    {"rtttheoretical", globemodule_rtttheoretical, METH_VARARGS, 
        globemodule_rtttheoretical_doc},
    {"interpoint", globemodule_interpoint, METH_VARARGS, 
        globemodule_interpoint_doc},
    {"degreestodms", globemodule_degreestodms, METH_VARARGS,
        globemodule_degreestodms_doc},
    {"dmstodegrees", globemodule_dmstodegrees, METH_VARARGS,
        globemodule_dmstodegrees_doc},
    {"disttoline", globemodule_disttoline, METH_VARARGS, 
        globemodule_disttoline_doc},
    /* sentinel */
    {NULL, NULL, 0, NULL}
};

PyDoc_STRVAR(globemodule_doc,
"utilities to calculate distances and angles on the Earth globe");

PyMODINIT_FUNC
initglobe(void)
{
    PyObject *m = NULL;

    m = Py_InitModule3("globe", globemodule_methods, globemodule_doc);
    if (m == NULL)
        return;
}
