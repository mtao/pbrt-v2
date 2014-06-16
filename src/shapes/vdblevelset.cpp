
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



// VDBLevelset Method Definitions
VDBLevelset::VDBLevelset(const Transform *o2w, const Transform *w2o,
        bool ro, const std::string& filename)
    : Shape(o2w, w2o, ro) {

    openvdb::io::File file(filename.c_str());
    file.open();
    openvdb::GridBase::Ptr baseGrid;
    for(openvdb::io::File::NameIterator nameIter = file.beginName(); nameIter != file.endName(); ++nameIter) {
       baseGrid = file.readGridMetadata(nameIter.gridName());
       if(baseGrid->isType<openvdb::FloatGrid>()) {
           this->gridPtr = openvdb::gridPtrCast<openvdb::FloatGrid>(file.readGrid(nameIter.gridName()));
           break;
       }
    }
    lsri = new openvdb::tools::LevelSetRayIntersector<openvdb::FloatGrid>(*gridPtr);
}


VDBLevelset::~VDBLevelset() {
    delete lsri;
}


BBox VDBLevelset::ObjectBound() const {
    openvdb::CoordBBox bbox = gridPtr->evalActiveVoxelBoundingBox();
    openvdb::Vec3d min = gridPtr->indexToWorld(bbox.min());
    openvdb::Vec3d max = gridPtr->indexToWorld(bbox.max());
    return BBox(Point(min[0],min[1],min[2]), Point(max[0],max[1],max[2]));
}


bool VDBLevelset::Intersect(const Ray &r, float *tHit, float *rayEpsilon,
                         DifferentialGeometry *dg) const {
    Ray ray;
    (*WorldToObject)(r,&ray);

    typedef openvdb::math::Ray<double> VDBRay;
    typedef VDBRay::Vec3Type VDBVec3;

    VDBVec3 dir(ray.d[0],ray.d[1],ray.d[2]);
    dir.normalize();
    const VDBVec3 eye(ray.o[0],ray.o[1],ray.o[2]);
    VDBVec3 w,n;
    VDBRay vdb_ray(eye,dir,ray.mint,ray.maxt);
    double time;
    if(lsri->intersectsWS(vdb_ray,w,n,time)) {
        *tHit = time;
        *rayEpsilon = 5e-5 * (*tHit);//TODO: Set this to a better value?
        n.normalize();
        const Normal pbrt_n(n.x(),n.y(),n.z());
        Vector dpdu(Normalize(Cross(pbrt_n,Vector(1,0,0))));
        if(dpdu.LengthSquared() < 0.5) {
            dpdu = Normalize(Cross(pbrt_n,Vector(0,1,0)));
        }
        const Vector dpdv(Normalize(Cross(pbrt_n,dpdu)));
                
        *dg = DifferentialGeometry(r(*tHit), (*ObjectToWorld)(dpdu), (*ObjectToWorld)(dpdv),
                Normal(0,0,0), Normal(0,0,0), 0,0, this);
        dg->nn = pbrt_n;

        return true;
    } else {
        return false;
    }

}

bool VDBLevelset::IntersectP(const Ray &r) const {
    Ray ray;
    (*WorldToObject)(r,&ray);

    typedef openvdb::math::Ray<double> VDBRay;
    typedef VDBRay::Vec3Type VDBVec3;

    VDBVec3 dir(ray.d[0],ray.d[1],ray.d[2]);
    dir.normalize();
    const VDBVec3 eye(ray.o[0],ray.o[1],ray.o[2]);
    VDBRay vdb_ray(eye,dir,ray.mint,ray.maxt);
    return lsri->intersectsWS(vdb_ray);
}



VDBLevelset *CreateVDBLevelsetShape(const Transform *o2w, const Transform *w2o,
        bool reverseOrientation, const ParamSet &params) {
    std::string filename = params.FindOneString("filename","");
    return new VDBLevelset(o2w, w2o, reverseOrientation, filename);
}


