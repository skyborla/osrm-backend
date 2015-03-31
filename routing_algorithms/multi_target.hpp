/*

Copyright (c) 2014, Project OSRM, Felix Guendling
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.
Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef MULTI_TARGET_ROUTING_H
#define MULTI_TARGET_ROUTING_H

#include "routing_base.hpp"
#include "../data_structures/search_engine_data.hpp"
#include "../typedefs.h"

#include <boost/assert.hpp>

#include <memory>
#include <vector>

template <class DataFacadeT, bool forward>
class MultiTargetRouting final : public BasicRoutingInterface<DataFacadeT, MultiTargetRouting<DataFacadeT, forward>>
{
    typedef BasicRoutingInterface<DataFacadeT, MultiTargetRouting<DataFacadeT, forward>> super;
    typedef SearchEngineData::QueryHeap QueryHeap;
    SearchEngineData &engine_working_data;

  public:
    MultiTargetRouting(DataFacadeT *facade, SearchEngineData &engine_working_data)
        : super(facade), engine_working_data(engine_working_data)
    {
    }

    ~MultiTargetRouting() {}

    std::shared_ptr<std::vector<std::pair<EdgeWeight, double>>>
    operator()(const PhantomNodeArray &phantom_nodes_array) const
    {
        BOOST_ASSERT(phantom_nodes_array.size() >= 2);

        // Prepare results table:
        // Each target (phantom_nodes_array[1], ..., phantom_nodes_array[n]) has its own index.
        auto results = std::make_shared<std::vector<std::pair<EdgeWeight, double>>>();
        results->reserve(phantom_nodes_array.size() - 1);

        // For readability: some reference variables making it clear,
        // which one is the source and which are the targets.
        const auto &source = phantom_nodes_array[0];
        std::vector<std::reference_wrapper<const std::vector<PhantomNode>>> targets(
            std::next(begin(phantom_nodes_array)), end(phantom_nodes_array));

        engine_working_data.InitializeOrClearFirstThreadLocalStorage(
            super::facade->GetNumberOfNodes());

        // The forward heap keeps the distances from the source.
        // Therefore it will be reused for each target backward search.
        QueryHeap &forward_heap = *(engine_working_data.forward_heap_1);

        // The reverse heap will be cleared after each search from
        // one of the targets to the source.
        QueryHeap &reverse_heap = *(engine_working_data.reverse_heap_1);

        // Fill forward heap with the source location phantom node(s).
        // The source location is located at index 0.
        // The target locations are located at index [1, ..., n].
        EdgeWeight min_edge_offset = 0;
        for (const PhantomNode &phantom_node : source)
        {
            if (SPECIAL_NODEID != phantom_node.forward_node_id)
            {
                EdgeWeight offset = (forward ? -1 : 1) * phantom_node.GetForwardWeightPlusOffset();
                forward_heap.Insert(phantom_node.forward_node_id, offset,
                                    phantom_node.forward_node_id);
                min_edge_offset = std::min(min_edge_offset, offset);
            }
            if (SPECIAL_NODEID != phantom_node.reverse_node_id)
            {
                EdgeWeight offset = (forward ? -1 : 1) * phantom_node.GetReverseWeightPlusOffset();
                forward_heap.Insert(phantom_node.reverse_node_id,
                                    (forward ? -1 : 1) * phantom_node.GetReverseWeightPlusOffset(),
                                    phantom_node.reverse_node_id);
                min_edge_offset = std::min(min_edge_offset, offset);
            }
        }

        for (auto const &target : targets)
        {
            const std::vector<PhantomNode> &phantom_node_vector = target;
            auto result = FindShortestPath(source, phantom_node_vector, forward_heap, reverse_heap,
                                           min_edge_offset);
            results->emplace_back(std::move(result));
        }

        forward_heap.Clear();
        reverse_heap.Clear();

        return results;
    }

    std::pair<EdgeWeight, double> FindShortestPath(const std::vector<PhantomNode> &source,
                                                   const std::vector<PhantomNode> &target,
                                                   QueryHeap &forward_heap,
                                                   QueryHeap &backward_heap,
                                                   EdgeWeight min_edge_offset) const
    {
        NodeID middle = UINT_MAX;
        int local_upper_bound = INT_MAX;

        // Clear backward heap from the entries produced by the search to the last target
        // and initialize heap for this target.
        backward_heap.Clear();
        for (const PhantomNode &phantom_node : target)
        {
            if (SPECIAL_NODEID != phantom_node.forward_node_id)
            {
                EdgeWeight offset = (forward ? 1 : -1) * phantom_node.GetForwardWeightPlusOffset();
                backward_heap.Insert(phantom_node.forward_node_id, offset,
                                     phantom_node.forward_node_id);
                min_edge_offset = std::min(min_edge_offset, offset);
            }

            if (SPECIAL_NODEID != phantom_node.reverse_node_id)
            {
                EdgeWeight offset = (forward ? 1 : -1) * phantom_node.GetReverseWeightPlusOffset();
                backward_heap.Insert(phantom_node.reverse_node_id, offset,
                                     phantom_node.reverse_node_id);
                min_edge_offset = std::min(min_edge_offset, offset);
            }
        }

        // Execute bidirectional Dijkstra shortest path search.
        while (0 < backward_heap.Size() || 0 < forward_heap.Size())
        {
            if (0 < forward_heap.Size())
            {
                super::RoutingStep(forward_heap, backward_heap, &middle, &local_upper_bound,
                                   min_edge_offset, forward, false);
            }
            if (0 < backward_heap.Size())
            {
                super::RoutingStep(backward_heap, forward_heap, &middle, &local_upper_bound,
                                   min_edge_offset, !forward);
            }
        }

        // Check if no path could be found (-> early exit).
        if (INVALID_EDGE_WEIGHT == local_upper_bound)
        {
            return std::make_pair(INVALID_EDGE_WEIGHT, 0);
        }

        // Calculate exact distance in km.
        std::vector<NodeID> packed_path;
        super::RetrievePackedPathFromHeap(forward_heap, backward_heap, middle, packed_path);

        if (!forward)
        {
            std::reverse(begin(packed_path), end(packed_path));
        }

        auto source_phantom_it =
            std::find_if(begin(source), end(source), [&packed_path](PhantomNode const &node)
                         {
                return node.forward_node_id == packed_path[forward ? 0 : packed_path.size() - 1] ||
                       node.reverse_node_id == packed_path[forward ? 0 : packed_path.size() - 1];
            });
        auto target_phantom_it =
            std::find_if(begin(target), end(target), [&packed_path](PhantomNode const &node)
                         {
                return node.forward_node_id == packed_path[forward ? packed_path.size() - 1 : 0] ||
                       node.reverse_node_id == packed_path[forward ? packed_path.size() - 1 : 0];
            });

        BOOST_ASSERT(source_phantom_it != end(source));
        BOOST_ASSERT(target_phantom_it != end(target));

        std::vector<PathData> unpacked_path;
        if (forward)
        {
            super::UnpackPath(packed_path, {*source_phantom_it, *target_phantom_it}, unpacked_path);
        }
        else
        {
            super::UnpackPath(packed_path, {*target_phantom_it, *source_phantom_it}, unpacked_path);
        }

        std::vector<FixedPointCoordinate> coordinates;
        coordinates.reserve(unpacked_path.size() + 2);

        coordinates.emplace_back(forward ? source_phantom_it->location
                                         : target_phantom_it->location);
        for (const auto &path_data : unpacked_path)
        {
            coordinates.emplace_back(super::facade->GetCoordinateOfNode(path_data.node));
        }
        coordinates.emplace_back(forward ? target_phantom_it->location
                                         : source_phantom_it->location);

        double distance = 0.0;
        for (size_t i = 1; i < coordinates.size(); ++i)
        {
            distance += coordinate_calculation::euclidean_distance(coordinates[i - 1],
                                                                   coordinates[i]);
        }

        return std::make_pair(round(local_upper_bound / 10.), distance);
    }
};

#endif
