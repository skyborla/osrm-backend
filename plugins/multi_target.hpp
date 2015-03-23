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

#ifndef MULTI_TARGET_PLUGIN_H
#define MULTI_TARGET_PLUGIN_H

#include "./plugin_base.hpp"

#include "../algorithms/object_encoder.hpp"
#include "../data_structures/search_engine.hpp"
#include "../Util/json_renderer.hpp"
#include "../Util/timing_util.hpp"

template <class DataFacadeT, bool forward> class MultiTargetPlugin final : public BasePlugin
{
  public:
    explicit MultiTargetPlugin(DataFacadeT *facade)
        : facade(facade), search_engine_ptr(std::make_shared<SearchEngine<DataFacadeT>>(facade))
    {
    }

    virtual ~MultiTargetPlugin() {}

    std::shared_ptr<std::vector<std::pair<EdgeWeight, double>>>
    HandleRequest(const RouteParameters &route_parameters,
                  unsigned &calctime_in_us)
    {
        // check number of parameters
        if (2 > route_parameters.coordinates.size())
        {
            return nullptr;
        }

        if (std::any_of(begin(route_parameters.coordinates), end(route_parameters.coordinates),
                        [&](FixedPointCoordinate coordinate)
                        {
                return !coordinate.is_valid();
            }))
        {
            return nullptr;
        }

        const bool checksum_OK = (route_parameters.check_sum == facade->GetCheckSum());
        PhantomNodeArray phantom_node_vector(route_parameters.coordinates.size());
        for (unsigned i = 0; i < route_parameters.coordinates.size(); ++i)
        {
            if (checksum_OK && i < route_parameters.hints.size() &&
                !route_parameters.hints[i].empty())
            {
                PhantomNode current_phantom_node;
                ObjectEncoder::DecodeFromBase64(route_parameters.hints[i], current_phantom_node);
                if (current_phantom_node.is_valid(facade->GetNumberOfNodes()))
                {
                    phantom_node_vector[i].emplace_back(std::move(current_phantom_node));
                    continue;
                }
            }
            facade->IncrementalFindPhantomNodeForCoordinate(route_parameters.coordinates[i],
                                                            phantom_node_vector[i],
                                                            1);

            BOOST_ASSERT(phantom_node_vector[i].front().is_valid(facade->GetNumberOfNodes()));
        }

        std::shared_ptr<std::vector<std::pair<EdgeWeight, double>>> ret;

        TIMER_START(multi_target);
        if (forward)
        {
            ret = search_engine_ptr->multi_target(phantom_node_vector);
        }
        else
        {
            ret = search_engine_ptr->multi_source(phantom_node_vector);
        }
        TIMER_STOP(multi_target);
        calctime_in_us = TIMER_USEC(multi_target);

        return ret;
    }

    int HandleRequest(const RouteParameters &route_parameters,
                      osrm::json::Object &json_result)
    {
        unsigned calctime_in_ms = 0;
        auto result_table = HandleRequest(route_parameters, calctime_in_ms);

        if (!result_table)
        {
            return 400;
        }

        osrm::json::Array json_array;
        for (unsigned column = 0; column < route_parameters.coordinates.size() - 1; ++column)
        {
            auto routing_result = result_table->operator[](column);

            osrm::json::Object result;
            result.values["time_cost"] = routing_result.first;
            result.values["distance"] = routing_result.second;
            json_array.values.emplace_back(result);
        }
        json_result.values["distances"] = json_array;
        return 200;
    }

    const std::string GetDescriptor() const { return forward ? "multitarget" : "multisource"; }

  private:
    DataFacadeT *facade;
    std::shared_ptr<SearchEngine<DataFacadeT>> search_engine_ptr;
};

#endif // MULTI_TARGET_PLUGIN_H
