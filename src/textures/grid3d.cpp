
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


// textures/grid3d.cpp*
#include "stdafx.h"
#include "textures/grid3d.h"
#include <iostream>

// CheckerboardTexture Method Definitions
Texture<float> *CreateGrid3DFloatTexture(const Transform &tex2world,
        const TextureParams &tp) {
    Warning("Creating a 3dgrid float texture");

    const ParamSet& gp = tp.GetGeomParams();
    const float default_value = tp.FindFloat("default_value",0);
    int ndims;//has to be 3
    const int * dims = gp.FindInt("dims",&ndims);
    if(ndims != 3) {
        Error("Dimensions for a 3D grid must have 3 ints");
    }
    int nx,ny,nz;
    nx = dims[0];
    ny = dims[1];
    nz = dims[2];


    int N;
    const float * data = gp.FindFloat("data",&N);
    if( N != nx * ny * nz ) {
        Error("Dimensions and grid elements dont match");
    }


    // Initialize 3D texture mapping _map_ from _tp_
    TextureMapping3D *map = new IdentityMapping3D(tex2world);
    return new Grid3DTexture<float>(map, nx,ny,nz,data,default_value);
}

Texture<Spectrum> *CreateGrid3DSpectrumTexture(const Transform &tex2world,
        const TextureParams &tp) {
    int dim = tp.FindInt("dimension", 2);

    const ParamSet& gp = tp.GetGeomParams();
    const float default_value = tp.FindFloat("default",0);
    if (dim != 2 && dim != 3) {
        Error("%d dimensional checkerboard texture not supported", dim);
        return NULL;
    }
    int ndims;//has to be 3
    const int * dims = gp.FindInt("dims",&ndims);
    if(ndims != 3) {
        Error("Dimensions for a 3D grid must have 3 ints");
    }
    int nx,ny,nz;
    nx = dims[0];
    ny = dims[1];
    nz = dims[2];

    int N;
    const Spectrum* data = gp.FindSpectrum("data",&N);
    if( N == nx * ny * nz ) {
        Error("Dimensions and grid elements dont match");
    }


    // Initialize 3D texture mapping _map_ from _tp_
    TextureMapping3D *map = new IdentityMapping3D(Inverse(tex2world));
    return new Grid3DTexture<Spectrum>(map, nx,ny,nz,data,default_value);
}
