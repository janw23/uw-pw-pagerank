#ifndef SRC_MULTITHREADEDPAGERANKCOMPUTER_HPP_
#define SRC_MULTITHREADEDPAGERANKCOMPUTER_HPP_

#include <atomic>
#include <mutex>
#include <thread>

#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "immutable/network.hpp"
#include "immutable/pageIdAndRank.hpp"
#include "immutable/pageRankComputer.hpp"

#include "singleThreadedPageRankComputer.hpp" // todo RT
#include <iostream> // todo RT

// NOTATKI:
// https://en.cppreference.com/w/cpp/container#Thread_safety
// metody wstępnego haszhowania stron:
// - przydzielenie każdemu wątkowi takiej samej liczby stron do shaszhowania
//   (jeśli jakiś wątek dostanie większe strony, to inne możliwe że będą na niego czekać)
//   (może przydzielenie robić losowo?)
// - pobieranie zadań z kolejki przez gotowe wątki
//   (wymaga synchronizacji między wątkami)
// można wcześniej przygotować struktury danych, każdą przez jeden wątek?
// metody dostępu do poprzednich wartości rang stron, które linkują do danej strony:

// todo hide it as private class member
struct PageInfo {
    // using struct should optimize caching and omit use of multiple maps

    PageInfo() = default;
    PageInfo(PageRank initialRank, size_t numLinks, bool isDangling)
        : ranks { initialRank, initialRank }
        , numLinks(numLinks)
        , isDangling(isDangling)
    {
    }

    /// Sets current rank to provided value. Returns the difference
    /// between the previous rank (which it does not update!).
    PageRank setCurrentRank(uint32_t iteration, PageRank newRank) noexcept
    {
        const bool odd = iteration % 2;
        ranks[odd] = newRank;
        return newRank - ranks[1 - odd];
    }

    /// Returns value of a single link from the page, calculated
    /// for the previous value of PageRank.
    PageRank getLinkValue(uint32_t iteration) const noexcept
    {
        return ranks[1 - iteration % 2] / numLinks;
    }

    PageRank getCurrentRank(uint32_t iteration)
    {
        return ranks[iteration % 2];
    }

    PageRank ranks[2]; // current and prev ranks, depending on iteration parity todo OK?
    size_t numLinks; // todo original impl has uint32_t, why?
    bool isDangling;
};

class MultiThreadedPageRankComputer : public PageRankComputer {
public:
    MultiThreadedPageRankComputer(uint32_t numThreadsArg)
        : numThreads(numThreadsArg) {};

    std::vector<PageIdAndRank>
    computeForNetwork(Network const& network, double alpha, uint32_t iterations,
        double tolerance) const
    {
        // todo parallelize
        for (auto const& page : network.getPages()) {
            page.generateId(network.getGenerator());
        }

        std::unordered_map<PageId, PageInfo, PageIdHash> pageHashMap;
        std::unordered_map<PageId, std::vector<PageId>, PageIdHash> edges;

        size_t danglingCount = 0;
        PageRank initialRank = 1.0 / network.getSize();

        // todo parallelize
        for (auto page : network.getPages()) {
            size_t linksNum = page.getLinks().size();
            bool isDangling = (linksNum == 0);
            danglingCount += isDangling;

            pageHashMap[page.getId()] = PageInfo(initialRank, linksNum, isDangling);
        }

        // todo parallelize
        // Prepare this structure using synchronized hashmap access (prob better)
        // or keep this data in the struct and update online?
        for (auto page : network.getPages()) {
            for (auto link : page.getLinks()) {
                edges[link].push_back(page.getId());
            }
        }

        double dangleSum = initialRank * danglingCount;

        for (uint32_t iteration = 0; iteration < iterations; ++iteration) {
            double prevDangleSum = dangleSum;
            double difference = 0;

            for (auto& pageMapElem : pageHashMap) {
                PageId pageId = pageMapElem.first;
                PageInfo& pageInfo = pageMapElem.second;

                double danglingWeight = 1.0 / network.getSize();
                PageRank newRank = prevDangleSum * alpha * danglingWeight + (1.0 - alpha) / network.getSize();

                if (edges.count(pageId) > 0) {
                    for (auto link : edges[pageId]) {
                        newRank += alpha * pageHashMap[link].getLinkValue(iteration);
                    }
                }

                PageRank rankDifference = pageInfo.setCurrentRank(iteration, newRank);
                dangleSum += rankDifference * pageInfo.isDangling;

                difference += std::abs(rankDifference);
            }

            // todo Why is this updated in every iteration?
            std::vector<PageIdAndRank> result;
            for (auto iter : pageHashMap) {
                result.push_back(PageIdAndRank(iter.first, iter.second.getCurrentRank(iteration)));
            }

            ASSERT(result.size() == network.getSize(),
                   "Invalid result size=" << result.size() << ", for network" << network);

            if (difference < tolerance) {
                return result;
            }
        }

        ASSERT(false, "Not able to find result in iterations=" << iterations);
    }

    std::string getName() const
    {
        return "MultiThreadedPageRankComputer[" + std::to_string(this->numThreads) + "]";
    }

private:
    uint32_t numThreads;
};

#endif /* SRC_MULTITHREADEDPAGERANKCOMPUTER_HPP_ */
