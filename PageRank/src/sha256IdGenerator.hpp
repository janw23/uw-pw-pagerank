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
        // todo maybe command can be created more efficiently?
        std::string command;
        command.reserve(content.size() + 30);
        command.append("printf \"").append(content).append("\" | sha256sum");

        auto pd = popen(command.data(), "r");

        const uint hash_length = 64; // sha256 hash has fixed character length
        char buffer[hash_length + 1]; // todo Is it okay? Maybe put it as class member
        buffer[hash_length] = '\0';

        fgets(buffer, hash_length + 1, pd); // todo Why does it need to be hashlength + 1?
        pclose(pd);

        return std::string(buffer);
    }
};

#endif /* SRC_SHA256IDGENERATOR_HPP_ */
