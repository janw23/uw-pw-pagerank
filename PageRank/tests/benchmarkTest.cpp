#include "../src/immutable/common.hpp"
#include "../src/immutable/pageIdAndRank.hpp"

#include "../src/multiThreadedPageRankComputer.hpp"
#include "../src/singleThreadedPageRankComputer.hpp"

#include "./lib/networkGenerator.hpp"
#include "./lib/performanceTimer.hpp"
#include "./lib/resultVerificator.hpp"
#include "./lib/simpleIdGenerator.hpp"

void pageRankComputationWithNumNodes(uint32_t num, PageRankComputer const &computer,
                                     NetworkGenerator const &networkGenerator) {
    Network network = networkGenerator.generateNetworkOfSize(num);
    PerformanceTimer timer;
    std::vector<PageIdAndRank> result = computer.computeForNetwork(network, 0.85, 100, 0.0000001);
    timer.printTimeDifference(
            "PageRank Performance Test [" + std::to_string(num) + " nodes, " + computer.getName() +
            "]");

    ASSERT(result.size() == network.getSize(), "Invalid result size=" << result.size());
}

int main(int argc, char **argv) {
    // program args are: program name, threads count (omit for single threaded impl)
    ASSERT(argc <= 2, "Too many arguments: " << argc);

    SingleThreadedPageRankComputer computer;
    SimpleIdGenerator simpleIdGenerator(
            "2000f1ffa5ce95d0f1e1893598e6aeeb2c214c85a88e3569d62c2dccd06a8725");
    SimpleNetworkGenerator simpleNetworkGenerator(simpleIdGenerator);
    NetworkWithoutManyEdgesGenerator networkWithoutEdgesGenerator(simpleIdGenerator);

    std::shared_ptr<PageRankComputer> computerPtr;
    if (argc == 1) {
        computerPtr = std::make_shared<SingleThreadedPageRankComputer>();
    } else {
        uint32_t numThreads;
        std::stringstream(argv[1]) >> numThreads;
        computerPtr = std::make_shared<MultiThreadedPageRankComputer>(numThreads);
    }

    pageRankComputationWithNumNodes(100, *computerPtr, simpleNetworkGenerator);
    pageRankComputationWithNumNodes(1000, *computerPtr, simpleNetworkGenerator);
    pageRankComputationWithNumNodes(2000, *computerPtr, simpleNetworkGenerator);
    pageRankComputationWithNumNodes(500000, *computerPtr, networkWithoutEdgesGenerator);

    return 0;
}
