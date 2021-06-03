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
#ifndef __MAPS_SURFACEPATCHES_HPP_
#define __MAPS_SURFACEPATCHES_HPP_

#include <numeric/PlaneFitting.hpp>

#include <base/Eigen.hpp>
#include <base-logging/Logging.hpp>

#include <Eigen/Core>

#include <boost/serialization/access.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/version.hpp>

#include <boost_serialization/BaseNumeric.hpp>

#include "MLSConfig.hpp"
#include "Index.hpp"
#include <cmath>
#include <limits>
#include <vector>


namespace maps { namespace grid
{
//! global typedef for all SurfacePatches. Only float seems to be useful.
typedef float SPScalar;
typedef Eigen::Matrix<SPScalar, 3, 1> Vector3;
typedef Eigen::Matrix<SPScalar, 3, 3> Matrix33;


class SurfacePatchBase
{
protected:
    float min, max;

public:
    // TODO set default value properly
    SurfacePatchBase()
        : min(0),
          max(0)
    {}

    explicit SurfacePatchBase(const float &z)
    : min(z), max(z)
    {}

    explicit SurfacePatchBase(const float &mean, const float &height)
    : min(mean - height), max(mean)
    {}

    bool merge(const SurfacePatchBase& other, const float& gapSize=0.0f)
    {
        if(!isCovered(other, gapSize)) return false;
        min = std::min(min, other.min);
        max = std::max(max, other.max);
        return true;
    }

    float getMin() const { return min; }
    float getMax() const { return max; }
    float getTop() const { return max; }
    float getBottom() const { return min; }
    void getRange(float& mini, float& maxi) const {mini = min; maxi = max; }

    bool isCovered(const float& z, const float& gapSize = 0.0f) const
    {
        return min - gapSize < z && z < max + gapSize;
    }

    bool isCovered(const SurfacePatchBase& other, const float& gapSize) const
    {
        return min - gapSize < other.max && other.min - gapSize < max;
    }

    bool operator<(const SurfacePatchBase& other) const
    {
        return min < other.min;
    }

    bool operator==(const SurfacePatchBase& other) const
    {
        return min == other.min && max == other.max;
    }

    void getClosestContactPoint(const Vector3& pos_in_cell, Vector3& contact_point) const
    {
        if(pos_in_cell.z() > max)
            contact_point << pos_in_cell.block(0,0,2,1), max;
        else if(pos_in_cell.z() < min)
            contact_point << pos_in_cell.block(0,0,2,1), min;
        else
            contact_point << pos_in_cell;
    }

    float getSurfacePos(const Vector3& pos_in_cell) const
    {
        return max;
    }

protected:
    /** Grants access to boost serialization */
    friend class boost::serialization::access;

    /** Serializes the members of this class*/
    template <typename Archive>
    void serialize(Archive &ar, const unsigned int version)
    {
        ar & BOOST_SERIALIZATION_NVP(min);
        ar & BOOST_SERIALIZATION_NVP(max);

    }
};

/**
 * Templated surface patch class.
 * TODO add color option
 */
template<MLSConfig::update_model model>
class SurfacePatch;

template<>
class SurfacePatch<MLSConfig::BASE> : public SurfacePatchBase
{
    typedef SurfacePatchBase Base;
public:
    explicit SurfacePatch(const float &z = 0, const float height = 0) : SurfacePatchBase(z, height) {}
    // implicit conversion from any other SurfacePatch:
    SurfacePatch(const SurfacePatchBase& other) : SurfacePatchBase(other) { }

    Eigen::Vector3f getCenter() const
    {
        Eigen::Vector3f center(0.0f, 0.0f, 0.5f*(max+min));
        return center;
    }

    Eigen::Vector3f getNormal() const
    {
        return Eigen::Vector3f::UnitZ();
    }

    bool merge(const SurfacePatch& other, const MLSConfig& config)
    {
        return Base::merge(other, config.gapSize);
    }
};


/**
 * SurfacePatch type for SLOPE update model.
 */
template<>
class SurfacePatch<MLSConfig::SLOPE> : public SurfacePatchBase
{
    typedef SurfacePatchBase Base;
    numeric::PlaneFitting<float> plane;
    float n;

public:

    SurfacePatch() : n(0)
    {}

    SurfacePatch(const Eigen::Vector3f& point, const float& cov)
        : Base(point.z())
        , plane(point, 1.0f/cov)
        , n(1)
    {}

    /**
     * Compares two patches by their center of mass
     */
    bool operator<(const SurfacePatch& other) const
    {
        return plane.z * other.plane.n < other.plane.z * plane.n;
    }

    bool operator==(const SurfacePatch& other) const
    {
        return Base::operator ==(other) &&
                plane.n == other.plane.n && plane.x == other.plane.x &&
                plane.y == other.plane.y && plane.z == other.plane.z &&
                plane.xx == other.plane.xx && plane.xy == other.plane.xy &&
                plane.xz == other.plane.xz && plane.yy == other.plane.yy &&
                plane.yz == other.plane.yz && plane.zz == other.plane.zz &&
                n == other.n;
    }

    /**
     * returns the top most
     */
    float getTop() const
    {
        // TODO
        return Base::getMax();
    }

    float getBottom() const
    {
        // TODO
        return Base::getMin();
    }

    Eigen::Vector3f getCenter() const
    {
        Eigen::Vector3f center(plane.x, plane.y, plane.z);
        return center * (1.0f/plane.n);
    }

    Eigen::Vector3f getNormal() const
    {
        if(n<=1.0f) return Eigen::Vector3f::UnitZ();
        return plane.getNormal();
    }

    bool merge(const SurfacePatch& other, const MLSConfig& config)
    {
        if(Base::merge(other, config.gapSize))
        {
            plane.update(other.plane);
            n+= other.n;
            return true;
        }

        return false;
    }

    float getClosestContactPoint(const Vector3& pos_in_cell, Vector3& contact_point) const
    {
        Eigen::Hyperplane<float, 3> plane(getNormal(), getCenter());
        const float distance = plane.signedDistance(pos_in_cell);
        contact_point = plane.projection(pos_in_cell);
        return distance;
    }

    float getSurfacePos(const Vector3& pos_in_cell) const
    {
        Eigen::Hyperplane<float, 3> plane(getNormal(), getCenter());
        float z_pos = (-plane.coeffs()(0) * pos_in_cell(0) - plane.coeffs()(1) * pos_in_cell(1) - plane.coeffs()(3)) / plane.coeffs()(2);
        if(z_pos > max) 
            z_pos = max;
        return z_pos;
    }

    int getNumerOfMeasurements() const
    {
        return n;
    }

protected:
    /** Grants access to boost serialization */
    friend class boost::serialization::access;

    /** @deprecated type definition */
    enum TYPE
    {
        VERTICAL = 0,
        HORIZONTAL = 1,
        NEGATIVE = 2
    };

    /** Serializes the members of this class*/
    template <typename Archive>
    void serialize(Archive &ar, const unsigned int version)
    {
        ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(SurfacePatchBase);
        ar & BOOST_SERIALIZATION_NVP(plane);
        ar & BOOST_SERIALIZATION_NVP(n);
        if(version == 0)
        {
            TYPE type;
            ar & BOOST_SERIALIZATION_NVP(type);
        }
    }    
}; // SurfacePatch<MLSConfig::SLOPE>

template<>
class SurfacePatch<MLSConfig::KALMAN>: public SurfacePatchBase
{
    typedef SurfacePatchBase Base;
    float mean, var, height;

    template <class T> static inline void kalman_update( T& mean, T& var, T m_mean, T m_var )
    {
        float gain = var / (var + m_var);
        if( gain != gain )
            gain = 0.5f; // this happens when both vars are 0.
        mean = mean + gain * (m_mean - mean);
        var = (1.0f-gain)*var;
    }


public:
    SurfacePatch() : mean(0), var(0), height(0)
    {}

    SurfacePatch(const Eigen::Vector3f& point, const float& variance)
        : Base(point.z())
        , mean(point.z()), var(variance), height(0)
    {}

    SurfacePatch(const float& mean, const float& variance, const float& height = 0)
        : Base(mean, height)
        , mean(mean), var(variance), height(height)
    {}

    bool merge(const SurfacePatch& other, const MLSConfig& config)
    {
        float delta_dev = std::sqrt(var + other.var);

        if(!Base::merge(other, config.gapSize + delta_dev))
            return false;

        if(isHorizontal() && other.isHorizontal() &&
            (mean - config.thickness - delta_dev) < other.mean &&
            (mean + config.thickness + delta_dev) > other.mean )
        {
            kalman_update(mean, var, other.mean, other.var);
        }
        else
        {
            height = std::max(mean, other.mean) - std::min(mean - height, other.mean - other.height);
            if(mean < other.mean)
            {
                mean = other.mean;
                var = other.var;
            }
        }

        // update min and max after current mean and height are computed
        min = mean - height;
        max = mean;

        return true;
    }

    Eigen::Vector3f getCenter() const
    {
        Eigen::Vector3f center(0.0f, 0.0f, mean);
        return center;
    }

    bool operator<(const SurfacePatch& other) const
    {
        return mean < other.mean;
    }

    bool operator==(const SurfacePatch& other) const
    {
        return Base::operator ==(other) && mean == other.mean && var == other.var && height == other.height;
    }

    Eigen::Vector3f getNormal() const
    {
        return Eigen::Vector3f::UnitZ();
    }

    float getMean() const
    {
        return mean;
    }

    float getHeight() const
    {
        return height;
    }

    float getVariance() const
    {
        return var;
    }

    float getStandardDeviation() const
    {
        return std::sqrt(var);
    }

    bool isVertical() const
    {
        return height != 0.0;
    }

    bool isHorizontal() const
    {
        return height == 0.0;
    }

    float getClosestContactPoint(const Vector3& pos_in_cell, Vector3& contact_point) const
    {
        const float std_dev = getStandardDeviation();
        const float z = pos_in_cell.z();
        if(z > max + std_dev)
        {
            contact_point << pos_in_cell.head<2>(), max + std_dev;
            return z - (max+std_dev);
        }
        else if(z < min - std_dev)
        {
            contact_point << pos_in_cell.head<2>(), min - std_dev;
            return z - (min-std_dev);
        }
        else
        {
            contact_point = pos_in_cell;
            return 0.f;
        }
    }

    float getSurfacePos(const Vector3& pos_in_cell) const
    {
        float std_dev = getStandardDeviation();
        return max + std_dev;
    }

protected:
    /** Grants access to boost serialization */
    friend class boost::serialization::access;

    /** Serializes the members of this class*/
    template <typename Archive>
    void serialize(Archive &ar, const unsigned int version)
    {
        ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(SurfacePatchBase);
        ar & BOOST_SERIALIZATION_NVP(mean);
        ar & BOOST_SERIALIZATION_NVP(var);
        ar & BOOST_SERIALIZATION_NVP(height);
    }

}; // SurfacePatch<MLSConfig::KALMAN>


/**
 * SurfacePatch type for data exchange
 */
template<>
class SurfacePatch<MLSConfig::PRECALCULATED> : public SurfacePatchBase
{
    typedef SurfacePatchBase Base;
    Eigen::Hyperplane<float, 3> plane;
public:
    
    SurfacePatch() : SurfacePatchBase()
    {
        // empty
    }

    SurfacePatch(const Eigen::Vector3f& center, const Eigen::Vector3f& normal, const float &min, const float &max) :
                SurfacePatchBase(), plane(normal, center)
    {
        this->min = min;
        this->max = max;
    }
    
    template<MLSConfig::update_model model>
    SurfacePatch(const SurfacePatch<model>& other) : SurfacePatchBase(other), plane(other.getNormal(), other.getCenter())
    {
        // empty
    }
    template<MLSConfig::update_model model>
    SurfacePatch& operator=(const SurfacePatch<model>& other)
    {
        new(this) SurfacePatch(other);
        return *this;
    }

    Eigen::Vector3f getCenter() const { return plane.normal() * -plane.offset(); }
    Eigen::Vector3f getNormal() const { return plane.normal(); }
    const Eigen::Vector4f& getCoeffs() const { return plane.coeffs(); }
    Eigen::Vector4f& getCoeffs() { return plane.coeffs(); }
    const Eigen::Hyperplane<float, 3>& getPlane() const { return plane; }
    Eigen::Hyperplane<float, 3>& getPlane() { return plane; }

    bool operator<(const SurfacePatch& other) const
    {
        return plane.offset() < other.plane.offset();
    }

    bool operator==(const SurfacePatch& other) const
    {
        return Base::operator==(other) && plane.coeffs() == other.plane.coeffs();
    }

    float getClosestContactPoint(const Vector3& pos_in_cell, Vector3& contact_point) const
    {
        const float distance = plane.signedDistance(pos_in_cell);
        contact_point = plane.projection(pos_in_cell);
        return distance;
    }

    float getSurfacePos(const Vector3& pos_in_cell) const
    {
        float z_pos = (-plane.coeffs()(0) * pos_in_cell(0) - plane.coeffs()(1) * pos_in_cell(1) - plane.coeffs()(3)) / plane.coeffs()(2);
        if(z_pos > max)
            z_pos = max;
        return z_pos;
    }

protected:
    /** Grants access to boost serialization */
    friend class boost::serialization::access;

    /** Serializes the members of this class*/
    template<class Archive>
    void save(Archive & ar, const unsigned int version) const
    {
        ar << BOOST_SERIALIZATION_BASE_OBJECT_NVP(SurfacePatchBase);
        ar << boost::serialization::make_nvp("plane_coeffs", plane.coeffs());
    }

    template<class Archive>
    void load(Archive & ar, const unsigned int version)
    {
        ar >> BOOST_SERIALIZATION_BASE_OBJECT_NVP(SurfacePatchBase);
        if(version == 0)
        {
            Eigen::Vector3f center, normal;
            ar >> boost::serialization::make_nvp("center", center);
            ar >> boost::serialization::make_nvp("normal", normal);
            plane = Eigen::Hyperplane<float, 3>(normal, center);
        }
        else
        {
            ar >> boost::serialization::make_nvp("plane_coeffs", plane.coeffs());
        }

    }

    BOOST_SERIALIZATION_SPLIT_MEMBER()
}; // SurfacePatch<MLSConfig::PRECALCULATED>



/**
 * Given the width of the cells, this outputs the bounding polygon points when intersecting
 * the fitting plane with the cell boundaries.
 * Optionally, it returns the normal vector, as well.
 */
template<MLSConfig::update_model S>
void getPolygon(std::vector<Eigen::Vector3f>& points, const SurfacePatch<S> &sp, const Eigen::Vector2f& posi, const Eigen::Vector2f& cell_size, Eigen::Vector3f * normal_out = 0)
{
    Eigen::Vector3f normal = sp.getNormal();
    if(normal_out) *normal_out = normal;
    float mn, mx;
    sp.getRange(mn,mx);
    Eigen::Vector3f extents, position;
    extents << 0.5f*cell_size, 0.5f*(mx-mn);
    position << posi, 0.5f*(mn+mx);

    Eigen::Vector3f mean = sp.getCenter();
    mean.z() -= position.z();

#ifndef NDEBUG
    // Sanity check (disabled in release mode)
    for(int i=0; i<3; ++i)
    {
        if( std::abs(mean[i]) > extents[i])
        {
            LOG_DEBUG_S << "Mean of SurfacePatch is outside of its extents! {";
            for(int j=0; j<3; ++j) LOG_DEBUG_S << mean[j] << (j==2? "}   {" : ", ");
            for(int j=0; j<3; ++j) LOG_DEBUG_S << extents[j] << (j==2? "}\n" : ", ");
            break;
        }
    }
#endif


    points.clear(); points.reserve(6);

    // first calculate the intersections of the plane with the borders of the box
    // Here, `extents` and `mean` are relative to the origin of the box
    float dist = mean.dot(normal); // scalar product gives the signed distance from the origin

    // find the max coefficient of the normal:
    int i=0, j=1, k=2;
    if(std::abs(normal[i]) < std::abs(normal[j])) std::swap(i,j);
    if(std::abs(normal[i]) < std::abs(normal[k])) std::swap(i,k);

    float dotj = extents[j]*normal[j];
    float dotk = extents[k]*normal[k];

    Eigen::Vector3f prev_p;
    enum { NONE, LOW, BOX, HIGH } prev_pos = NONE, pos;
    // calculate intersections in direction k:
    for(int n=0; n<5; ++n)
    {
        Eigen::Vector3f p(0,0,0);
        float dotp = 0.0f;
        if((n+1)&2)
            dotp += dotj, p[j] = extents[j];
        else
            dotp -= dotj, p[j] = -extents[j];
        if(n&2)
            dotp += dotk, p[k] = extents[k];
        else
            dotp -= dotk, p[k] = -extents[k];

        p[i] = (dist - dotp) / normal[i];

        if( p[i] < -extents[i])
            pos = LOW;
        else if( p[i] > extents[i])
            pos = HIGH;
        else
            pos = BOX;

        if( (prev_pos == LOW || prev_pos == HIGH) && pos != prev_pos )
        {
            // clipping in
            float h = prev_pos == LOW ? -extents[i] : extents[i];
            float s = (h - prev_p[i]) / (p[i] - prev_p[i]);
            Eigen::Vector3f cp = prev_p + (p - prev_p) * s;
            points.push_back(position + cp);
        }
        if( pos == BOX )
        {
            // plane intersected with outer square
            // for mostly horizontal patches this is the standard case
            if(n!=4) // do not push again, if we started here
                points.push_back(position + p);
        }
        else if( pos != prev_pos && prev_pos != NONE )
        {
            // clipping out
            float h = pos == LOW ? -extents[i] : extents[i];
            float s = (h - prev_p[i]) / (p[i] - prev_p[i]);
            Eigen::Vector3f cp = prev_p + (p - prev_p) * s;
            points.push_back(position + cp);
        }

        prev_pos = pos;
        prev_p = p;
    }
}

/**
 * Given the width of the cells, this outputs the bounding polygon points when intersecting
 * the fitting plane with the cell boundaries.
 * This function calculates the cells position using the Index vector (in local grid coordinates).
 * Optionally, it returns the normal vector, as well.
 */
template<MLSConfig::update_model S>
void getPolygon(std::vector<Eigen::Vector3f>& points, const SurfacePatch<S> &sp, const Index& idx, const Eigen::Vector2f& cell_size, Eigen::Vector3f * normal_out = 0)
{
    Eigen::Vector2f posi((idx.x()+0.5f)*cell_size.x(), (idx.y()+0.5f)*cell_size.y());
    getPolygon(points, sp, posi, cell_size, normal_out);
}


}  // namespace grid
}  // namespace maps


BOOST_CLASS_VERSION(maps::grid::SurfacePatch<maps::grid::MLSConfig::SLOPE>, 1);
BOOST_CLASS_VERSION(maps::grid::SurfacePatch<maps::grid::MLSConfig::PRECALCULATED>, 1);

#endif /* __MAPS_SURFACEPATCHES_HPP_ */
