#include "pchassiscollision.h"
#include <algorithm>
#include <cmath>

PChassisCollision::PChassisCollision(dBodyID chassisBody, dReal radius, dReal height, dReal dribblerDistance)
    : PObject(0, 0, 0, 0, 0, 0, 0), chassisBody(chassisBody), radius(radius), height(height),
      dribblerDistance(dribblerDistance)
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
    // Start with the full-detail body, then clip its front behind the configured
    // dribbler. Keep the original cutout when it already clears the dribbler.
    // Physics geometry stays fixed when render quality changes.
    const unsigned sides = 24, first = sides / 8, vertices = sides - 2*first + 1;
    const dReal front = std::min(dReal(radius*std::cos(M_PI/4)), std::max(dReal(0), dribblerDistance));
    const dReal epsilon = radius * 1e-6;
    auto append = [this, epsilon](dReal x, dReal y) {
        const auto size = points.size();
        if (!size || std::hypot(points[size-3]-x, points[size-2]-y) > epsilon)
            points.insert(points.end(), {x, y, -height/2});
    };
    for (unsigned i = 0; i < vertices; ++i) {
        const double angle = 2*M_PI*(first+i)/sides;
        const double nextAngle = 2*M_PI*(first+(i+1)%vertices)/sides;
        const dReal x = radius*std::cos(angle), y = radius*std::sin(angle);
        const dReal nextX = radius*std::cos(nextAngle), nextY = radius*std::sin(nextAngle);
        if (x <= front) append(x, y);
        if ((x <= front) != (nextX <= front)) {
            const dReal t = (front-x)/(nextX-x);
            append(front, y+t*(nextY-y));
        }
    }
    const auto size = points.size();
    if (std::hypot(points[0]-points[size-3], points[1]-points[size-2]) <= epsilon)
        points.resize(size-3);
    const unsigned count = points.size()/3;
    for (unsigned i = 0; i < count; ++i)
        points.insert(points.end(), {points[3*i], points[3*i+1], height/2});

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
