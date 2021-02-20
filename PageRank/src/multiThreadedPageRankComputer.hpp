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

#include <future>

class MultiThreadedPageRankComputer : public PageRankComputer {
public:
    MultiThreadedPageRankComputer(uint32_t numThreadsArg)
        : numThreads(numThreadsArg) {};

    std::vector<PageIdAndRank>
    computeForNetwork(Network const& network, double alpha,
        uint32_t iterations, double tolerance) const
    {

        Pool pool(numThreads);
        generatePageIds(pool, network);

        PageMap pageHashMap;
        EdgeMap edges;

        initEdges(pool, network, edges);

        size_t danglingCount = 0;
        PageRank initialRank = 1.0 / network.getSize();
        initPages(network, pageHashMap, initialRank, danglingCount);
        double dangleSum = initialRank * danglingCount;

        for (uint32_t iteration = 0; iteration < iterations; ++iteration) {
            auto changes = updateRanks(pool, pageHashMap, edges,
                network.getSize(), dangleSum, alpha, iteration);

            auto difference = changes.first;
            auto dangleSumChange = changes.second;
            dangleSum += dangleSumChange;

            if (difference < tolerance) {
                return generateResult(pageHashMap, network, iteration);
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

    struct Pool {
        Pool(uint32_t numThreads)
            : numThreads(numThreads)
            , threads() {};

        // [func]'s argument must be threadId.
        template <typename F>
        void execute_void(F func)
        {
            threads.clear();

            for (uint32_t i = 0; i < numThreads; i++)
                threads.emplace_back(func, i);
        }

        void join()
        {
            for (auto& thread : threads)
                thread.join();
        }

        template <typename R, typename F>
        std::vector<std::future<R>> execute_returning(F func)
        {
            std::vector<std::future<R>> futures;

            for (uint32_t i = 0; i < numThreads; i++)
                futures.push_back(std::async(func, i));

            return futures;
        }

        uint32_t numThreads;
        std::vector<std::thread> threads;
    };

    struct PageInfo {
        // using struct should optimize caching and omit use of multiple maps

        PageInfo() = default;

        PageInfo(PageRank initialRank, size_t numLinks, bool isDangling)
            : ranks { initialRank, initialRank }
            , numLinks(numLinks)
            , isDangling(isDangling)
        {
        }

        PageRank setCurrentRank(uint32_t iteration, PageRank newRank)
        {
            const bool odd = iteration % 2;
            ranks[odd] = newRank;
            return newRank - ranks[1 - odd];
        }

        PageRank getLinkValue(uint32_t iteration) const
        {
            return ranks[1 - iteration % 2] / numLinks;
        }

        PageRank getCurrentRank(uint32_t iteration) const { return ranks[iteration % 2]; }

        PageRank ranks[2]; // current and prev ranks, depending on iteration parity
        size_t numLinks;
        bool isDangling;
    };

    struct EdgeInfo {
        std::vector<PageId> links;
        std::mutex mutex;

        void push_back(PageId elem)
        {
            std::lock_guard<std::mutex> lck(mutex);
            links.push_back(elem);
        }
    };

    template <typename I>
    struct AtomicIterator {
        AtomicIterator(I start, I end)
            : mutex()
            , current(start)
            , end(end) {};

        AtomicIterator(const AtomicIterator<I>&& other)
            : mutex()
            , current(std::move(other.current))
            , end(std::move(other.end)) {};

        I fetch_advance(size_t dist)
        {
            std::lock_guard<std::mutex> lck(mutex);

            I fetched = current;

            for (size_t i = 0; i < dist && current != end; i++)
                current++;

            return fetched;
        };

        std::mutex mutex;
        I current;
        I end;
    };

    using PageMap = std::unordered_map<PageId, PageInfo, PageIdHash>;
    using EdgeMap = std::unordered_map<PageId, EdgeInfo, PageIdHash>;

    static void generatePageIds(Pool& pool, Network const& network)
    {
        std::atomic<size_t> index(0);

        auto func = [&network, &index]([[maybe_unused]] uint32_t threadId) {
            auto& pages = network.getPages();
            auto& generator = network.getGenerator();
            size_t fetched;

            while ((fetched = index.fetch_add(1)) < pages.size()) {
                pages[fetched].generateId(generator);
            }
        };

        pool.execute_void(func);
        pool.join();
    }

    static void initPages(Network const& network, PageMap& pageHashMap,
        PageRank initialRank, size_t& danglingCount)
    {

        danglingCount = 0;

        // no point in multithreading, as only 1 thread at a time can write to hash map
        for (auto const& page : network.getPages()) {
            size_t linksNum = page.getLinks().size();
            bool isDangling = (linksNum == 0);
            danglingCount += isDangling;
            pageHashMap[page.getId()] = PageInfo(initialRank, linksNum, isDangling);
        }
    }

    static void initEdges(Pool& pool, Network const& network, EdgeMap& edges)
    {

        for (auto const& page : network.getPages()) {
            for (auto const& link : page.getLinks()) {
                edges[link];
            }
        }

        std::atomic<size_t> pageIndex(0);

        auto func = [&network, &edges, &pageIndex]([[maybe_unused]] uint32_t threadId) {
            auto const& pages = network.getPages();

            size_t fetchedPageIndex;
            while ((fetchedPageIndex = pageIndex.fetch_add(1)) < pages.size()) {
                auto const& page = pages[fetchedPageIndex];

                for (auto const& link : page.getLinks()) {
                    edges[link].push_back(page.getId());
                }
            }
        };

        pool.execute_void(func);
        pool.join();
    }

    static std::pair<double, double>
    updateRanks(Pool& pool, PageMap& pageHashMap, EdgeMap& edges,
        const size_t networkSize, const double dangleSum, const double alpha, const uint32_t iteration)
    {

        using MapIter = decltype(pageHashMap.begin());
        std::vector<AtomicIterator<MapIter>> threadsIters;

        auto numThreads = pool.numThreads;

        for (uint32_t i = 0; i < numThreads; i++) {
            AtomicIterator<MapIter> iter(pageHashMap.begin(), pageHashMap.end());
            iter.fetch_advance(i); // i-th thread's iter starts at i-th pos
            threadsIters.push_back(std::move(iter));
        }

        auto func = [&pageHashMap, &edges,
                        networkSize, dangleSum, alpha, iteration,
                        &threadsIters, numThreads](uint32_t threadId) {
            double dangleSumChange = 0;
            double pageRankCumulativeChange = 0;

            // Each thread executes its own job (advancing iterator through numThreads elements)
            // and then tries to help other threads with their jobs.
            // This approach hopes to reduce waiting time on mutexes.
            for (uint32_t offset = 0; offset < numThreads; offset++) {
                uint32_t threadJobId = (threadId + offset) % numThreads;

                MapIter fetchedIter;
                while ((fetchedIter = threadsIters[threadJobId].fetch_advance(numThreads)) != pageHashMap.end()) {
                    auto& pageMapElem = *fetchedIter;
                    PageId const& pageId = pageMapElem.first;
                    PageInfo& pageInfo = pageMapElem.second;

                    PageRank newRank = (dangleSum * alpha + (1.0 - alpha)) / networkSize;

                    if (edges.count(pageId) > 0) {
                        for (auto const& link : edges[pageId].links) {
                            newRank += alpha * pageHashMap[link].getLinkValue(iteration);
                        }
                    }

                    PageRank rankDifference = pageInfo.setCurrentRank(iteration, newRank);
                    dangleSumChange += rankDifference * pageInfo.isDangling;
                    pageRankCumulativeChange += std::abs(rankDifference);
                }
            }

            return std::make_pair(pageRankCumulativeChange, dangleSumChange);
        };

        double pageRankCumulativeChange = 0;
        double dangleSumChange = 0;

        for (auto& fut : pool.execute_returning<std::pair<double, double>>(func)) {
            auto changes = fut.get();
            pageRankCumulativeChange += changes.first;
            dangleSumChange += changes.second;
        }

        return std::make_pair(pageRankCumulativeChange, dangleSumChange);
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

#endif /* SRC_MULTITHREADEDPAGERANKCOMPUTER_HPP_ */
