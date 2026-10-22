#include "pchassiscollision.h"
#include <cmath>

PChassisCollision::PChassisCollision(dBodyID chassisBody, dReal radius, dReal height)
    : PObject(0, 0, 0, 0, 0, 0, 0), chassisBody(chassisBody), radius(radius), height(height)
{
    setVisibility(false);
}

PChassisCollision::~PChassisCollision()
{
    // Destroy the geom before its backing arrays. PObject owns no body here;
    // the chassis remains the sole owner of the shared ODE body.
    if (geom) { dGeomDestroy(geom); geom = nullptr; }
}

void PChassisCollision::init()
{
    // Match the full-detail rendered body, including its +/-45 degree front
    // cutout. Physics geometry stays fixed when render quality changes.
    const unsigned sides = 24, first = sides / 8, count = sides - 2*first + 1;
    for (int layer = 0; layer < 2; ++layer) {
        for (unsigned i = 0; i < count; ++i) {
            const double angle = 2*M_PI*(first+i)/sides;
            points.insert(points.end(), {radius*std::cos(angle), radius*std::sin(angle),
                                         layer ? height/2 : -height/2});
        }
    }

    // Caps are counterclockwise as viewed from outside the hull.
    planes.insert(planes.end(), {0, 0, 1, height/2, 0, 0, -1, height/2});
    polygons.push_back(count);
    for (unsigned i = 0; i < count; ++i) polygons.push_back(count+i);
    polygons.push_back(count);
    for (unsigned i = 0; i < count; ++i) polygons.push_back(count-1-i);

    // The closing boundary edge forms the flat face behind the dribbler.
    for (unsigned i = 0; i < count; ++i) {
        const unsigned next = (i+1)%count;
        const dReal x = points[3*i], y = points[3*i+1];
        const dReal dx = points[3*next]-x, dy = points[3*next+1]-y;
        const dReal length = std::hypot(dx, dy), nx = dy/length, ny = -dx/length;
        planes.insert(planes.end(), {nx, ny, 0, nx*x+ny*y});
        polygons.insert(polygons.end(), {4, i, next, count+next, count+i});
    }
    geom = dCreateConvex(nullptr, planes.data(), count+2, points.data(), 2*count, polygons.data());
    dGeomSetBody(geom, chassisBody);
    dSpaceAdd(space, geom);
}
