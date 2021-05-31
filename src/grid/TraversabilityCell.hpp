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
#ifndef __MAPS_TRAVERSABILITY_CELL_HPP_
#define __MAPS_TRAVERSABILITY_CELL_HPP_


#include <boost/serialization/access.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/export.hpp>
#include <stdint.h>

namespace maps { namespace grid
{
    /**
    * Type used in TraversabilityGrid as a cell type to hold the values of traversabilityClassId and probability for the respective cell of the grid.
    * @details: traversabilityClassId is cooresponds to a TraversabilityClass holding the actual traversability value,
    *           while probability is used as a measure of certainty for the traversability value.
    * */
    class TraversabilityCell
    {
    private:
        uint8_t traversabilityClassId;
        uint8_t probability;

    public:
        // Default constructor, sets traversability and probability to zero.
        TraversabilityCell();

        // Initializes the TraversabilityCell with the given values for traversabilityClassId and probability.
        TraversabilityCell(uint8_t traversabilityClassId, uint8_t probability);

        ~TraversabilityCell();

        void setTraversabilityClassId(uint8_t traversabilityClassId);
        uint8_t getTraversabilityClassId() const;

        void setProbability(uint8_t probability);
        uint8_t getProbability() const;

        bool operator==(const maps::grid::TraversabilityCell& other) const;
        bool operator!=(const TraversabilityCell& other) const;

    protected:

        /** Grants access to boost serialization */
        friend class boost::serialization::access;

        /** Serializes the members of this class*/
        template <typename Archive>
        void serialize(Archive &ar, const unsigned int version)
        {
        ar & BOOST_SERIALIZATION_NVP(traversabilityClassId);
        ar & BOOST_SERIALIZATION_NVP(probability);
        }
    };

}  //end namespace grid
}  //end namespace maps

#endif  //__MAPS_TRAVERSABILITY_CELL_HPP_
