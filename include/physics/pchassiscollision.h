#ifndef PCHASSISCOLLISION_H
#define PCHASSISCOLLISION_H

#include "pobject.h"
#include <vector>

// Ball-only hull attached to the chassis body. The existing cylinder continues
// to handle walls and ground without requiring ODE's optional convex-box support.
class PChassisCollision : public PObject
{
public:
    PChassisCollision(dBodyID chassisBody, dReal radius, dReal height);
    ~PChassisCollision() override;
    void init() override;
private:
    dBodyID chassisBody;
    dReal radius, height;
    // ODE retains these arrays for the lifetime of the convex geom.
    std::vector<dReal> planes, points;
    std::vector<unsigned int> polygons;
};

#endif
