
/*
    pbrt source code Copyright(c) 1998-2012 Matt Pharr and Greg Humphreys.

    This file is part of pbrt.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are
    met:

    - Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.

    - Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
    IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
    TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
    PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
    HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
    SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
    LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
    DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
    THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 */


// shapes/vdblevelset.cpp*
#include "stdafx.h"
#include "shapes/vdblevelset.h"
#include "shapes/trianglemesh.h"
#include "paramset.h"
#include <openvdb/LevelSetRayIntersector.h>



// VDBLevelset Method Definitions
VDBLevelset::VDBLevelset(const Transform *o2w, const Transform *w2o,
        bool ro, const std::string& filename)
    : Shape(o2w, w2o, ro) {

    openvdb::io::File file(filename.c_str());
    openvdb::GridBase::Ptr baseGrid;
    for(openvdb::io::File::NameIterator nameIter = file.beginName(); nameIter != file.endName(); ++nameIter) {
       baseGrid = file.readGridMetadata(nameIter.gridName());
       if(baseGrid->isType<openvdb::FloatGrid>()) {
           this->gridPtr = openvdb::gridPtrCast<openvdb::FloatGrid>(file.readGrid(nameIter.gridName()));
           break;
       }
    }
}


VDBLevelset::~VDBLevelset() {
}


BBox VDBLevelset::ObjectBound() const {
    openvdb::CoordBBox bbox = gridPtr->evalActiveVoxelBoundingBox();
    openvdb::Vec3d min = gridPtr->indexToWorld(bbox.min());
    openvdb::Vec3d max = gridPtr->indexToWorld(bbox.max());
    return BBox(Point(min[0],min[1],min[2]), Point(max[0],max[1],max[2]));
}


bool VDBLevelset::Intersect(const Ray &ray, float *tHit, float *rayEpsilon,
                         DifferentialGeometry *dg) const {
    Ray ray;
    (*WorldToObject)(r,&ray);
    openvdb::tools::LevelSetRayIntersector<openvdb::FloatGrid> lsri(*gridPtr);
    openvdb::tools::LinearIntersector<openvdb::FloatGrid> tester(*gridPtr);

    typedef openvdb::Ray<float> VDBRay;
    typedef VDBRay::Vec3Type VDBVec3;

    const VDBVec3 dir(static_cast<float*>(&ray.d));
    const VDBVec3 eye(static_cast<float*>(&ray.o));
    VDBVec3 w,n;
    VDBRay vdb_ray(eye,dir,ray.mint,ray.maxt);
    if(lsri.intersectWS(vdb_ray,tester,w,n)) {
        *tHit = tester.time
        *rayEpsilon = 5e-4 * *tHit;//TODO: Set this to a better value?
        n.normalize();
        const Vector pbrt_n(n.x(),n.y(),n.z());
        Vector dpdu(Normalize(Cross(pbrt_n.cross,Vector(1,0,0))));
        if(dpdu.LengthSquared() < 0.5) {
            dpdu = Normalize(Cross(pbrt_n,Vector(0,1,0)));
        }
        const Vector dpdv(Normalize(Cross(pbrt_n,dpdu)));
                
        *dg = DifferentialGeometry(o2w(Point(w.x(),w.y(),w.z())), o2w(dpdu), o2w(dpdv),
                o2w(dndu), o2w(dndv), u, v, this);
        return true;
    } else {
        return false;
    }

}

bool VDBLevelset::IntersectP(const Ray &ray) const {
    Ray ray;
    (*WorldToObject)(r,&ray);
    openvdb::tools::LevelSetRayIntersector<openvdb::FloatGrid> lsri(*gridPtr);
    openvdb::tools::LinearIntersector<openvdb::FloatGrid> tester(*gridPtr);

    typedef openvdb::Ray<float> VDBRay;
    typedef VDBRay::Vec3Type VDBVec3;

    const VDBVec3 dir(static_cast<float*>(&ray.d));
    const VDBVec3 eye(static_cast<float*>(&ray.o));
    VDBRay vdb_ray(eye,dir,ray.mint,ray.maxt);
    return lsri.intersectWS(vdb_ray,tester);
}



VDBLevelset *CreateVDBLevelsetShape(const Transform *o2w, const Transform *w2o,
        bool reverseOrientation, const ParamSet &params) {
    std::string filename = params.FindOneString("filename","");
    return new VDBLevelset(o2w, w2o, reverseOrientation, filename);
}


