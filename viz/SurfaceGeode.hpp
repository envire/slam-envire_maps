//
// Copyright (c) 2015-2017, Deutsches Forschungszentrum für Künstliche Intelligenz GmbH.
// Copyright (c) 2015-2017, University of Bremen
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
#ifndef __VIZKIT_SURFACEGEODE_HPP__
#define __VIZKIT_SURFACEGEODE_HPP__

#include <osg/Geometry>
#include <osg/Geode>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include <base-logging/Logging.hpp>

namespace vizkit3d {

    class SurfaceGeode : public osg::Geode
    {
    public:
        SurfaceGeode(float x_res, float y_res);

        void setColor(const osg::Vec4& color);
        void setColorHSVA(float hue, float sat, float lum, float alpha);

        void showCycleColor(bool cycle_color);
        void setCycleColorInterval(float cycle_color_interval);
        void setUncertaintyScale(double uncertainty_scale);

        void setShowPatchExtents(bool enable = true) { showPatchExtents = enable; };
        void setShowNormals(bool enable = true) { showNormals = enable; };
        void setShowUncertainty(bool enable = true) { showUncertainty = enable; }
        void drawLines();


        void addVertex(const osg::Vec3& p, const osg::Vec3& n, const float & stdev = 0.f);

        void closeTriangleStrip(){
            geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::TRIANGLE_STRIP,vertex_index,vertices->size()-vertex_index));
            vertex_index = vertices->size();
        }
        inline osg::ref_ptr<osg::Geometry> getGeometry(){
            return geom;
        }

    private:
        osg::ref_ptr<osg::Vec3Array> vertices;
        osg::ref_ptr<osg::Vec3Array> normals;
        osg::ref_ptr<osg::Vec4Array> colors;  
        osg::ref_ptr<osg::Geometry> geom;  

        osg::ref_ptr<osg::Vec3Array> var_vertices;

        size_t vertex_index;

        float xp, yp; // position of current patch
        float xs, ys; // grid resolution

        float hue;
        float sat; 
        float alpha; 
        float lum;
        osg::Vec4 color; 

        bool showNormals;
        bool showPatchExtents;
        bool showUncertainty;
        bool cycle_color;
        float cycle_color_interval;
        double uncertaintyScale;

        
        void updateColor();

    };

}

#endif // __VIZKIT_PATCHESGEODE_HPP__
