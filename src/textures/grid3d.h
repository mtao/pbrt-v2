
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

#if defined(_MSC_VER)
#pragma once
#endif

#ifndef PBRT_TEXTURES_GRID3D_H
#define PBRT_TEXTURES_GRID3D_H

// textures/grid3d.h*
#include "pbrt.h"
#include "texture.h"
#include "paramset.h"
#include "montecarlo.h"
#include "shape.h"
#include "parallel.h"
#include "progressreporter.h"



template <typename T> class Grid3DTexture : public Texture<T> {
public:
    // Grid3DTexture Public Methods
    Grid3DTexture(TextureMapping3D *m, int nx, int ny, int nz, const T* data_, T default_value)
        : mapping(m), nx(nx),ny(ny),nz(nz),default_value(default_value) {
            int N = nx*ny*nz;
            data = new T[N];
            std::copy(data_,data_+N,data);
    }
    ~Grid3DTexture() {
        delete mapping;
        delete[] data;
    }

    void barycentric(const Point& p, int ix, int iy, int iz, float dx, float dy, float dz) const {
        dx = p.x * nx;
        ix = int(dx);
        dx -=  ix;
        dy = p.y * ny;
        iy = int(dy);
        dy -=  iy;
        dz = p.z * nz;
        iz = int(dz);
        dz -=  iz;
    }

    const T& datagrid(int ix, int iy, int iz) const {
        return data[ix + nx * (iy + ny * (iz))];
    }

    T Evaluate(const DifferentialGeometry &dg) const {
        Vector dpdx, dpdy;
        Point p = mapping->Map(dg, &dpdx, &dpdy);
    float dx,dy,dz;
    int ix,iy,iz;
    barycentric(p,ix,iy,iz,dx,dy,dz);
    const T& w000 = datagrid(ix,   iy,   iz);//,   dx,   dy,   dz);
    const T& w100 = datagrid(ix+1, iy,   iz);//,   dx-1, dy,   dz);
    const T& w010 = datagrid(ix,   iy+1, iz);//,   dx,   dy-1, dz);
    const T& w110 = datagrid(ix+1, iy+1, iz);//,   dx-1, dy-1, dz);
    const T& w001 = datagrid(ix,   iy,   iz+1);//, dx,   dy,   dz-1);
    const T& w101 = datagrid(ix+1, iy,   iz+1);//, dx-1, dy,   dz-1);
    const T& w011 = datagrid(ix,   iy+1, iz+1);//, dx,   dy-1, dz-1);
    const T& w111 = datagrid(ix+1, iy+1, iz+1);//, dx-1, dy-1, dz-1);

    // Compute trilinear interpolation of weights
    T x00 = LerpT(dx, w000, w100);
    T x10 = LerpT(dx, w010, w110);
    T x01 = LerpT(dx, w001, w101);
    T x11 = LerpT(dx, w011, w111);
    T y0 = LerpT(dy, x00, x10);
    T y1 = LerpT(dy, x01, x11);
    return LerpT(dz, y0, y1);

    }
    
private:
    // Grid3DTexture Private Data
    TextureMapping3D *mapping;
    int nx,ny,nz;
    T * data;
    T default_value;
};

Texture<float> *CreateGrid3DFloatTexture(const Transform &tex2world, const TextureParams& tp);
Texture<Spectrum> *CreateGrid3DSpectrumTexture(const Transform &tex2world, const TextureParams& tp);
#endif // PBRT_TEXTURES_GRID3D_H
