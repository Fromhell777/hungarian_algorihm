#include "timer.hpp"
#include "prettyprint.hpp"

#include <cstddef>
#include <cstdint>
#include <iostream>
#include <climits>
#include <vector>
#include <utility>
#include <string>

std::string find_minimum_flight_assignment(
  std::vector<std::pair<std::uint32_t, std::uint32_t>> const & flights,
  std::vector<std::pair<std::uint32_t, std::uint32_t>> const & planes)
{
  // Check if there are enough planes available for the flights
  if (flights.size() > planes.size()) {return "geen oplossing";}

  // Create weighted edges of the bipartite graph
  std::vector<std::vector<std::pair<std::size_t, std::uint32_t>>> flights_edges(flights.size());
  for (std::size_t i = 0; i < flights.size(); ++i)
    {
      for (std::size_t j = 0; j < planes.size(); ++j)
        {
          if (planes[j].first  >= flights[i].first &&
              planes[j].second >= flights[i].second)
            {
              flights_edges[i].emplace_back(j, flights[i].first *
                                              (planes[j].second - flights[i].second));
            }
        }

      // Make sure there is at least one plain that can do this flight
      if (flights_edges[i].size() == 0) {return "geen oplossing";}
    }

  std::vector<std::int32_t> left_labels(flights.size(), 0);
  std::vector<std::int32_t> right_labels(planes.size(), 0);
  std::vector<std::int32_t> match_pairs(planes.size(), -1);
  std::vector<std::int32_t> back_trace(planes.size(), -1);

  std::uint32_t result = 0;

  for (std::size_t i = 0; i < flights.size(); ++i)
    {
      std::vector<std::uint32_t> min_weight_left_vertex(planes.size(), UINT_MAX);
      std::vector<bool> used_right_vertices(planes.size(), false);
      std::vector<bool> reachable_rigth_vertices(planes.size(), false);

      std::size_t current_left_vertex = i;
      std::int32_t current_right_vertex = -1;

      do {
        std::uint32_t delta = UINT_MAX;
        std::int32_t next_right_vertex = -1;

        // Changes minimum weights due to the newly available edges from the
        // current left vertex
        for (auto const & flight_edge : flights_edges[current_left_vertex])
          {
            std::size_t j = flight_edge.first;

            if (!used_right_vertices[j])
              {
                reachable_rigth_vertices[j] = true;

                std::uint32_t cur = flight_edge.second -
                                    left_labels[current_left_vertex] -
                                    right_labels[j];
                if (cur < min_weight_left_vertex[j])
                  {
                    min_weight_left_vertex[j] = cur;
                    back_trace[j] = current_right_vertex;
                  }
              }
          }

        // Search the minimum weight from the reachable rigth vertices that are
        // not yet used
        for (std::size_t j = 0; j < planes.size(); ++j)
          {
            if (!used_right_vertices[j] && reachable_rigth_vertices[j])
              {
                if (min_weight_left_vertex[j] < delta)
                  {
                    delta = min_weight_left_vertex[j];
                    next_right_vertex = j;
                  }
              }
          }

        // No pair was found
        if (next_right_vertex == -1) {return "geen oplossing";}

        // Update the labels
        left_labels[i] += delta;
        for (std::size_t j = 0; j < planes.size(); ++j)
          {
            if (reachable_rigth_vertices[j])
              {
                if (used_right_vertices[j])
                  {
                    left_labels[match_pairs[j]] += delta;
                    right_labels[j] -= delta;
                  }
                else
                  {
                    min_weight_left_vertex[j] -= delta;
                  }
              }
          }

        // Add the current delta to the result for the final minimal cost
        result += delta;

        current_right_vertex = next_right_vertex;

        used_right_vertices[current_right_vertex] = true;
        current_left_vertex = match_pairs[current_right_vertex];

      } while (match_pairs[current_right_vertex] != -1);

      while (back_trace[current_right_vertex] != -1) {
        std::size_t next_right_vertex = back_trace[current_right_vertex];
        match_pairs[current_right_vertex] = match_pairs[next_right_vertex];
        current_right_vertex = next_right_vertex;
      }

      match_pairs[current_right_vertex] = i;
    }

  return std::to_string(result);
}

int main()
{
  {
    timer timer;

    std::size_t test_cases;
    std::cin >> test_cases;

    for (std::size_t t = 0; t < test_cases; ++t)
      {
        std::size_t number_of_flights;
        std::cin >> number_of_flights;

        std::size_t number_of_planes;
        std::cin >> number_of_planes;

        std::vector<std::pair<std::uint32_t, std::uint32_t>> flights(number_of_flights);

        for (auto & flight : flights)
          {
            std::uint32_t flight_distance;
            std::cin >> flight_distance;

            std::uint32_t flight_passengers;
            std::cin >> flight_passengers;

            flight.first  = flight_distance;
            flight.second = flight_passengers;
          }

        std::vector<std::pair<std::uint32_t, std::uint32_t>> planes(number_of_planes);

        for (auto & plane : planes)
          {
            std::uint32_t plane_distance;
            std::cin >> plane_distance;

            std::uint32_t plane_passengers;
            std::cin >> plane_passengers;

            plane.first  = plane_distance;
            plane.second = plane_passengers;
          }

        std::cout << t + 1 << ' ' << find_minimum_flight_assignment(flights,
                                                                    planes) << '\n';
      }
  }
}
