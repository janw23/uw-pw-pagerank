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
        generatePageIds(network);

        PageMap pageHashMap;
        pageHashMap.reserve(network.getSize());

        size_t danglingCount = 0;
        const PageRank initialRank = 1.0 / network.getSize();
        initPages(network, pageHashMap, initialRank, danglingCount);
        double dangleSum = initialRank * danglingCount;

        initEdges(network, pageHashMap);

        for (uint32_t iteration = 0; iteration < iterations; ++iteration) {
            auto changes = updateRanks(pageHashMap,
                network.getSize(), dangleSum, alpha, iteration);

            auto difference = changes.first;
            dangleSum += changes.second;

            if (difference < tolerance) {
                return generateResult(pageHashMap, network, iteration);
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
            , links()
        {}

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
        size_t numLinks;
        bool isDangling;
        std::vector<PageId> links;

        void push_back(PageId const& elem)
        {
            links.push_back(elem);
        }
    };

    using PageMap = std::unordered_map<PageId, PageInfo, PageIdHash>;

    static void generatePageIds(Network const& network)
    {
        for (auto const& page : network.getPages()) {
            page.generateId(network.getGenerator());
        }
    }

    static void initPages(Network const& network, PageMap& pageHashMap,
        PageRank initialRank, size_t& danglingCount)
    {
        danglingCount = 0;

        for (auto const& page : network.getPages()) {
            size_t linksNum = page.getLinks().size();
            bool isDangling = (linksNum == 0);
            danglingCount += isDangling;
            pageHashMap[page.getId()] = PageInfo(initialRank, linksNum, isDangling);
        }
    }

    static void initEdges(Network const& network, PageMap & pageHashMap)
    {
        for (auto const& page : network.getPages()) {
            for (auto const& link : page.getLinks()) {
                pageHashMap[link].push_back(page.getId());
            }
        }
    }

    static std::pair<double, double>
    updateRanks(PageMap& pageHashMap,
        size_t networkSize, const double dangleSum, const double alpha, const uint32_t iteration)
    {

        double dangleSumChange = 0;
        double pageRankCumulativeChange = 0;

        for (auto& pageMapElem : pageHashMap) {
            PageInfo& pageInfo = pageMapElem.second;

            double danglingWeight = 1.0 / networkSize;
            PageRank newRank = dangleSum * alpha * danglingWeight + (1.0 - alpha) / networkSize;

            if (pageInfo.links.size() > 0) {
                for (auto const& link : pageInfo.links) {
                    newRank += alpha * pageHashMap[link].getLinkValue(iteration);
                }
            }

            PageRank rankDifference = pageInfo.setCurrentRank(iteration, newRank);
            dangleSumChange += rankDifference * pageInfo.isDangling;
            pageRankCumulativeChange += std::abs(rankDifference);
        }

        return { pageRankCumulativeChange, dangleSumChange };
    }

    static std::vector<PageIdAndRank>
    generateResult(PageMap const& pageHashMap, Network const& network, uint32_t iteration)
    {
        std::vector<PageIdAndRank> result;

        for (auto iter : pageHashMap) {
            result.push_back(PageIdAndRank(iter.first, iter.second.getCurrentRank(iteration)));
        }

        ASSERT(result.size() == network.getSize(),
            "Invalid changes size=" << result.size() << ", for network" << network);

        return result;
    }
};

#endif /* SRC_SINGLETHREADEDPAGERANKCOMPUTER_HPP_ */
