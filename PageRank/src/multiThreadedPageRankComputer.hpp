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

#include <iostream> // todo RT
#include <future> // todo RT???

// NOTATKI:
// https://en.cppreference.com/w/cpp/container#Thread_safety
// metody wstępnego haszhowania stron:
// - przydzielenie każdemu wątkowi takiej samej liczby stron do shaszhowania
//   (jeśli jakiś wątek dostanie większe strony, to inne możliwe że będą na
//   niego czekać) (może przydzielenie robić losowo?)
// - pobieranie zadań z kolejki przez gotowe wątki
//   (wymaga synchronizacji między wątkami)
// można wcześniej przygotować struktury danych, każdą przez jeden wątek?
// metody dostępu do poprzednich wartości rang stron, które linkują do danej
// strony:

// todo REMOVE START
class Timer {
public:
    void start() {
        m_StartTime = std::chrono::system_clock::now();
        m_bRunning = true;
    }

    void stop() {
        m_EndTime = std::chrono::system_clock::now();
        m_bRunning = false;
    }

    double elapsedNanoseconds() {
        std::chrono::time_point<std::chrono::system_clock> endTime;

        if (m_bRunning) {
            endTime = std::chrono::system_clock::now();
        } else {
            endTime = m_EndTime;
        }

        return std::chrono::duration_cast<std::chrono::nanoseconds>(endTime - m_StartTime).count();
    }

    double elapsedMicorseconds() {
        return elapsedNanoseconds() / 1000.0;
    }

    double elapsedMilliseconds() {
        return elapsedMicorseconds() / 1000.0;
    }

    double elapsedSeconds() {
        return elapsedMilliseconds() / 1000.0;
    }

private:
    std::chrono::time_point<std::chrono::system_clock> m_StartTime;
    std::chrono::time_point<std::chrono::system_clock> m_EndTime;
    bool m_bRunning = false;
};

template<typename F>
void measure_time(F func, std::string name) {
    Timer timer;
    timer.start();
    func();
    timer.stop();
    std::cerr << name << " took: " << timer.elapsedMilliseconds() << "ms"
              << "\n";
}

// todo REMOVE END

// todo Put it in better place
struct EdgeInfo {
    std::vector<PageId> links;
    std::mutex mutex;

    void push_back(PageId elem) {
        std::lock_guard<std::mutex> lck(mutex);
        links.push_back(elem);
    }
};

void generatePageIds_sequential(Network const &network) {
    for (auto &page : network.getPages()) {
        page.generateId(network.getGenerator());
    }
}

void generatePageIds_concurrent(Network const &network, uint32_t numThreads) {
    // It's just very slightly worse than strided on evenly sized data, but much
    // better on unevenly distributed.

    std::atomic<size_t> index(0);

    auto func = [&network, &index] {
        auto &pages = network.getPages();
        auto &generator = network.getGenerator();
        size_t fetched;

        while ((fetched = index.fetch_add(1)) < pages.size()) {
            pages[fetched].generateId(generator);
        }
    };

    std::vector<std::thread> threads;
    for (uint32_t i = 0; i < numThreads; i++)
        threads.emplace_back(func);
    for (auto &thread : threads)
        thread.join();
}

template<typename E>
void initEdges_sequential(Network const &network,
                          std::unordered_map<PageId, E, PageIdHash> &edges) {
    for (auto const &page : network.getPages()) {
        for (auto const &link : page.getLinks()) {
            edges[link].push_back(page.getId());
        }
    }
}

template<typename E>
void initEdges_concurrent(Network const &network,
                          std::unordered_map<PageId, E, PageIdHash> &edges,
                          uint32_t numThreads) {

    for (auto const &page : network.getPages()) {
        for (auto const &link : page.getLinks()) {
            edges[link];
        }
    }

    std::atomic<size_t> pageIndex(0);

    auto func = [&network, &edges, &pageIndex] {
        auto const &pages = network.getPages();

        size_t fetchedPageIndex;
        while ((fetchedPageIndex = pageIndex.fetch_add(1)) < pages.size()) {
            auto const &page = pages[fetchedPageIndex];

            for (auto const &link : page.getLinks()) {
                edges[link].push_back(page.getId());
            }
        }
    };

    std::vector<std::thread> threads;
    for (uint32_t i = 0; i < numThreads; i++)
        threads.emplace_back(func);
    for (auto &thread : threads)
        thread.join();
}

class MultiThreadedPageRankComputer : public PageRankComputer {
public:
    MultiThreadedPageRankComputer(uint32_t numThreadsArg)
            : numThreads(numThreadsArg) {};

    std::vector<PageIdAndRank>
    computeForNetwork(Network const &network, double alpha,
                      uint32_t iterations, double tolerance) const {

        // todo write cleaner code
        generatePageIds_concurrent(network, numThreads);

        std::unordered_map<PageId, PageInfo, PageIdHash> pageHashMap;
        std::unordered_map<PageId, EdgeInfo, PageIdHash> edges;

        size_t danglingCount = 0;
        const PageRank initialRank = 1.0 / network.getSize();

        // no point in multithreading, as only 1 thread at a time can write to hash map
        // todo But this can be done by main thread while executing edges creation??
        for (auto const &page : network.getPages()) {
            size_t linksNum = page.getLinks().size();
            bool isDangling = (linksNum == 0);
            danglingCount += isDangling;
            pageHashMap[page.getId()] = PageInfo(initialRank, linksNum, isDangling);
        }

        initEdges_concurrent(network, edges, numThreads);

        double dangleSum = initialRank * danglingCount;

        for (uint32_t iteration = 0; iteration < iterations; ++iteration) {
            double prevDangleSum = dangleSum;
            double difference = 0;

            for (auto &pageMapElem : pageHashMap) {
                PageId pageId = pageMapElem.first;
                PageInfo &pageInfo = pageMapElem.second;

                double danglingWeight = 1.0 / network.getSize();
                PageRank newRank =
                        prevDangleSum * alpha * danglingWeight + (1.0 - alpha) / network.getSize();

                if (edges.count(pageId) > 0) {
                    for (auto const& link : edges[pageId].links) {
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

    std::string getName() const {
        return "MultiThreadedPageRankComputer[" + std::to_string(this->numThreads) + "]";
    }

private:
    uint32_t numThreads;

    struct PageInfo {
        // using struct should optimize caching and omit use of multiple maps

        PageInfo() = default;

        PageInfo(PageRank initialRank, size_t numLinks, bool isDangling)
                : ranks{initialRank, initialRank}, numLinks(numLinks), isDangling(isDangling) {
        }

        PageRank setCurrentRank(uint32_t iteration, PageRank newRank) noexcept {
            const bool odd = iteration % 2;
            ranks[odd] = newRank;
            return newRank - ranks[1 - odd];
        }

        PageRank getLinkValue(uint32_t iteration) const {
            return ranks[1 - iteration % 2] / numLinks;
        }

        PageRank getCurrentRank(uint32_t iteration) const noexcept { return ranks[iteration % 2]; }

        PageRank ranks[2]; // current and prev ranks, depending on iteration parity
        size_t numLinks; // todo original impl has uint32_t, why?
        bool isDangling;
    };
};

#endif /* SRC_MULTITHREADEDPAGERANKCOMPUTER_HPP_ */
