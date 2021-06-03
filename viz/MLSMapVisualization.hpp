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
#ifndef maps_MLSMapVisualization_H
#define maps_MLSMapVisualization_H

#include <vizkit3d/MapVisualization.hpp>

#include <osg/Geode>
#include <osg/Shape>
#include <osg/Texture2D>

#if QT_VERSION >= 0x050000 || !defined(Q_MOC_RUN)
    #include <maps/grid/MLSMap.hpp>
#endif

namespace vizkit3d
{
    class PatchesGeode;

    class MLSMapVisualization
        : public vizkit3d::MapVisualization< ::maps::grid::MLSMapKalman >
        , public vizkit3d::VizPluginAddType< ::maps::grid::MLSMap<::maps::grid::MLSConfig::SLOPE> >
        , public vizkit3d::VizPluginAddType< ::maps::grid::MLSMap<::maps::grid::MLSConfig::PRECALCULATED> >
        , public vizkit3d::VizPluginAddType< ::maps::grid::MLSMap<::maps::grid::MLSConfig::BASE> >
    {
        Q_OBJECT

        Q_PROPERTY(bool show_map_extents READ areMapExtentsShown WRITE setShowMapExtents)
        Q_PROPERTY(bool show_uncertainty READ isUncertaintyShown WRITE setShowUncertainty)
        Q_PROPERTY(bool show_negative READ isNegativeShown WRITE setShowNegative)
        Q_PROPERTY(bool show_patch_extents READ arePatchExtentsShown WRITE setShowPatchExtents)
        Q_PROPERTY(bool estimate_normals READ areNormalsEstimated WRITE setEstimateNormals)
        Q_PROPERTY(bool show_normals READ areNormalsShown WRITE setShowNormals)
        Q_PROPERTY(bool cycle_height_color READ isHeightColorCycled WRITE setCycleHeightColor)
        Q_PROPERTY(double cycle_color_interval READ getCycleColorInterval WRITE setCycleColorInterval)
        Q_PROPERTY(double uncertainty_scale READ getUncertaintyScale WRITE setUncertaintyScale)
        Q_PROPERTY(bool connected_surface READ isConnectedSurface WRITE setConnectedSurface)
        Q_PROPERTY(bool simplify_surface READ getSimplifySurface WRITE setSimplifySurface)
        Q_PROPERTY(bool connected_surface_lod READ getConnectedSurfaceLOD WRITE setConnectedSurfaceLOD)
        Q_PROPERTY(QColor horizontal_cell_color READ getHorizontalCellColor WRITE setHorizontalCellColor)
        Q_PROPERTY(QColor vertical_cell_color READ getVerticalCellColor WRITE setVerticalCellColor)
        Q_PROPERTY(QColor negative_cell_color READ getNegativeCellColor WRITE setNegativeCellColor)
        Q_PROPERTY(QColor uncertainty_color READ getUncertaintyColor WRITE setUncertaintyColor)
        Q_PROPERTY(int min_measurements READ getMinMeasurements WRITE setMinMeasurements)

        public:
            MLSMapVisualization();
            ~MLSMapVisualization();

            void visualize(vizkit3d::PatchesGeode& geode, const maps::grid::SurfacePatch<maps::grid::MLSConfig::SLOPE>& p);
            void visualize(vizkit3d::PatchesGeode& geode, const maps::grid::SurfacePatch<maps::grid::MLSConfig::PRECALCULATED>& p);
            void visualize(vizkit3d::PatchesGeode& geode, const maps::grid::SurfacePatch<maps::grid::MLSConfig::KALMAN>& p);
            void visualize(vizkit3d::PatchesGeode& geode, const maps::grid::SurfacePatch<maps::grid::MLSConfig::BASE>& p);

            Q_INVOKABLE void updateMLSPrecalculated(maps::grid::MLSMapPrecalculated const &sample)
            {vizkit3d::Vizkit3DPlugin<::maps::grid::MLSMapKalman>::updateData(sample);}

            Q_INVOKABLE void updateMLSSloped(maps::grid::MLSMapSloped const &sample)
            {vizkit3d::Vizkit3DPlugin<::maps::grid::MLSMapKalman>::updateData(sample);}

            Q_INVOKABLE void updateMLSKalman(maps::grid::MLSMapKalman const &sample)
            {vizkit3d::Vizkit3DPlugin<::maps::grid::MLSMapKalman>::updateData(sample);}

            Q_INVOKABLE void updateMLSBase(maps::grid::MLSMapBase const &sample)
            {vizkit3d::Vizkit3DPlugin<::maps::grid::MLSMapKalman>::updateData(sample);}

            Q_INVOKABLE void updateData(maps::grid::MLSMapPrecalculated const &sample)
            {vizkit3d::Vizkit3DPlugin<::maps::grid::MLSMapKalman>::updateData(sample);}

            Q_INVOKABLE void updateData(maps::grid::MLSMapSloped const &sample)
            {vizkit3d::Vizkit3DPlugin<::maps::grid::MLSMapKalman>::updateData(sample);}

            Q_INVOKABLE void updateData(maps::grid::MLSMapKalman const &sample)
            {vizkit3d::Vizkit3DPlugin<maps::grid::MLSMapKalman>::updateData(sample);}

            // this is dirty fix to resolve typedef problem
            Q_INVOKABLE void updateMLSKalmanFull(maps::grid::MLSMap<maps::grid::MLSConfig::KALMAN> const &sample)
            { vizkit3d::Vizkit3DPlugin<::maps::grid::MLSMapKalman>::updateData(sample); }


        protected:
            virtual osg::ref_ptr<osg::Node> createMainNode();
            virtual void updateMainNode(osg::Node* node);
            virtual void updateDataIntern(maps::grid::MLSMapKalman const& mls);
            virtual void updateDataIntern(::maps::grid::MLSMap<::maps::grid::MLSConfig::SLOPE> const& mls);
            virtual void updateDataIntern(::maps::grid::MLSMap<::maps::grid::MLSConfig::PRECALCULATED> const& mls);
            virtual void updateDataIntern(::maps::grid::MLSMap<::maps::grid::MLSConfig::BASE> const& mls);

        private:
            struct Data;
            boost::scoped_ptr<Data> p;

            osg::ref_ptr<osg::Group> localNode;
        
        public slots:

            bool isUncertaintyShown() const;
            void setShowUncertainty(bool enabled);

            bool isNegativeShown() const;
            void setShowNegative(bool enabled);

            bool areNormalsEstimated() const;
            void setEstimateNormals(bool enabled);

            bool areNormalsShown() const;
            void setShowNormals(bool enabled);

            bool isHeightColorCycled() const;
            void setCycleHeightColor(bool enabled);

            double getCycleColorInterval() const;
            void setCycleColorInterval(double interval);

            QColor getHorizontalCellColor() const;
            void setHorizontalCellColor(QColor color);

            QColor getVerticalCellColor() const;
            void setVerticalCellColor(QColor color);

            QColor getNegativeCellColor() const;
            void setNegativeCellColor(QColor color);

            QColor getUncertaintyColor() const;
            void setUncertaintyColor(QColor color);

            void setShowPatchExtents( bool value );
            bool arePatchExtentsShown() const; 

            double getUncertaintyScale() const;
            void setUncertaintyScale(double scaling);

            bool isConnectedSurface() const;
            void setConnectedSurface(bool enabled);

            bool getSimplifySurface() const;
            void setSimplifySurface(bool enabled);

            bool getConnectedSurfaceLOD() const;
            void setConnectedSurfaceLOD(bool enabled);

            int getMinMeasurements() const;
            void setMinMeasurements(int measurements);

        protected:
            osg::Vec4 horizontalCellColor;
            osg::Vec4 verticalCellColor;
            osg::Vec4 negativeCellColor;
            osg::Vec4 uncertaintyColor;

            bool showUncertainty;
            bool showNegative;
            bool estimateNormals;
            bool showNormals;
            bool cycleHeightColor;
            double cycleColorInterval;
            bool showPatchExtents;
            double uncertaintyScale;
            int minMeasurements;
            bool connectedSurface;
            bool simplifySurface;
            bool connected_surface_lod;

#if 0
            osg::Vec3 estimateNormal(
                const ::maps::MLSMap &grid, 
                const ::maps::SurfacePatch &patch, 
                const ::maps::Index &patch_idx) const;
#endif
    };
}
#endif
