#ifndef SRC_SHA256IDGENERATOR_HPP_
#define SRC_SHA256IDGENERATOR_HPP_

#include <sys/wait.h>
#include <unistd.h>

#include "immutable/idGenerator.hpp"
#include "immutable/pageId.hpp"

class Sha256IdGenerator : public IdGenerator {
public:
    virtual PageId generateId(std::string const& content) const
    {
        std::string command;
        command.reserve(content.size() + 30);
        command.append("printf \"").append(content).append("\" | sha256sum");

        auto pd = popen(command.data(), "r");
        if (pd == nullptr) {
            throw std::runtime_error("popen failed");
        }

        const uint hash_length = 64; // sha256 hash has fixed character length
        char buffer[hash_length + 1];

        if (fgets(buffer, hash_length + 1, pd) == nullptr) { // +1 for terminating character
            pclose(pd);
            throw std::runtime_error("fgets failed");
        }

        if (pclose(pd) == -1) {
            throw std::runtime_error("pclose failed");
        }

        return std::string(buffer);
    }
};

#endif /* SRC_SHA256IDGENERATOR_HPP_ */
