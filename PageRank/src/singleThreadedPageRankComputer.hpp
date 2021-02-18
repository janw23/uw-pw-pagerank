#ifndef SRC_SINGLETHREADEDPAGERANKCOMPUTER_HPP_
#define SRC_SINGLETHREADEDPAGERANKCOMPUTER_HPP_

#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "immutable/network.hpp"
#include "immutable/pageIdAndRank.hpp"
#include "immutable/pageRankComputer.hpp"

class SingleThreadedPageRankComputer : public PageRankComputer {
public:
    SingleThreadedPageRankComputer() {};

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
        const PageRank initialRank = 1.0 / network.getSize();

        // todo parallelize
        for (auto const& page : network.getPages()) {
            size_t linksNum = page.getLinks().size();
            bool isDangling = (linksNum == 0);
            danglingCount += isDangling;
            pageHashMap[page.getId()] = PageInfo(initialRank, linksNum, isDangling);
        }

        // todo parallelize
        // Prepare this structure using synchronized hashmap access (prob better)
        // or keep this data in the struct and update online?
        for (auto const& page : network.getPages()) {
            for (auto const& link : page.getLinks()) {
                edges[link].push_back(page.getId());
            }
        }

        double dangleSum = initialRank * danglingCount;

        for (uint32_t iteration = 0; iteration < iterations; ++iteration) {
            double prevDangleSum = dangleSum;
            double difference = 0;

            for (auto& pageMapElem : pageHashMap) {
                PageId const& pageId = pageMapElem.first;
                PageInfo& pageInfo = pageMapElem.second;

                double danglingWeight = 1.0 / network.getSize();
                PageRank newRank = prevDangleSum * alpha * danglingWeight + (1.0 - alpha) / network.getSize();

                if (edges.count(pageId) > 0) {
                    for (auto const& link : edges[pageId]) {
                        newRank += alpha * pageHashMap[link].getLinkValue(iteration);
                    }
                }

                PageRank rankDifference = pageInfo.setCurrentRank(iteration, newRank);
                dangleSum += rankDifference * pageInfo.isDangling;

                difference += std::abs(rankDifference);
            }

            if (difference < tolerance) {
                std::vector<PageIdAndRank> result;
                for (auto iter : pageHashMap) {
                    result.push_back(
                        PageIdAndRank(iter.first, iter.second.getCurrentRank(iteration)));
                }

                ASSERT(result.size() == network.getSize(),
                    "Invalid result size=" << result.size() << ", for network"
                                           << network);

                return result;
            }
        }

        ASSERT(false, "Not able to find result in iterations=" << iterations);
    }

    std::string getName() const
    {
        return "SingleThreadedPageRankComputer";
    }

private:
    struct PageInfo {
        PageInfo() = default;

        PageInfo(PageRank initialRank, size_t numLinks, bool isDangling)
            : ranks { initialRank, initialRank }
            , numLinks(numLinks)
            , isDangling(isDangling)
        {
        }

        PageRank setCurrentRank(uint32_t iteration, PageRank newRank) noexcept
        {
            const bool odd = iteration % 2;
            ranks[odd] = newRank;
            return newRank - ranks[1 - odd];
        }

        PageRank getLinkValue(uint32_t iteration) const
        {
            return ranks[1 - iteration % 2] / numLinks;
        }

        PageRank getCurrentRank(uint32_t iteration) const noexcept { return ranks[iteration % 2]; }

        PageRank ranks[2];
        size_t numLinks; // todo original impl has uint32_t, why?
        bool isDangling;
    };
};

#endif /* SRC_SINGLETHREADEDPAGERANKCOMPUTER_HPP_ */
