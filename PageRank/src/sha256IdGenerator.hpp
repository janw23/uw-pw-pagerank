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

//        std::string command = "printf ";
//        command += "\"";
//        command += content;
//        command += "\" | sha256sum";
//
//        auto pd = popen(command.data(), "r");
//
//        const uint hash_length = 64; // sha256 hash has fixed character length
//        char buffer[hash_length + 1]; // todo Is it okay? Maybe put it as class member
//        buffer[hash_length] = '\0';
//
//        fgets(buffer, hash_length + 1, pd);
//        pclose(pd);
//
//        return std::string(buffer);

        // todo Implement pipes as RAII based on what I have here.

        Pipe p2c;
        Pipe c2p;

        pid_t pid = fork_wrap();

        if (pid == 0) {
            exec_child_path(std::move(p2c), std::move(c2p));
            throw std::runtime_error("exec_child_path() neither threw nor exited");
        } else {
            std::string hash = exec_parent_path(
                p2c, c2p, content);
            return PageId(std::move(hash));
        }
    }

private:
    class Pipe {
        // todo Write better impl
    public:
        enum class PipeEnd{READ = 0, WRITE = 1};

        Pipe() : closed{false, false} {
            pipe_wrap(pipedes);
        }

        ~Pipe() {
            close(PipeEnd::WRITE);
            close(PipeEnd::READ);
        }

        void close(PipeEnd end) {
            int end_index = static_cast<int>(end);
            if (!closed[end_index]) close_wrap(pipedes[end_index]);
        }

        void write(const void* buf, ssize_t nbytes) const {
            write_wrap(static_cast<int>(PipeEnd::WRITE), buf, nbytes);
        }

        void read(void* buf, size_t nbytes) const {
            read_wrap(static_cast<int>(PipeEnd::READ), buf, nbytes);
        }

        void dup(PipeEnd end, int fd) {
            dup2_wrap(pipedes[static_cast<int>(end)], fd);
        }

    private:
        int pipedes[2];
        bool closed[2];
    };

    void exec_child_path(Pipe && p2c, Pipe && c2p) const
    {

        p2c.dup(Pipe::PipeEnd::READ, STDIN_FILENO);
        c2p.dup(Pipe::PipeEnd::WRITE, STDOUT_FILENO);

        // todo close pipe explicitly?

        exec_wrap("sha256sum");
    }

    std::string exec_parent_path(Pipe & p2c, Pipe & c2p, std::string const& content) const
    {

        const uint hash_length = 64; // sha256 hash has fixed character length
        char buffer[hash_length + 1]; // todo Is it okay? Maybe put it as class member
        buffer[hash_length] = '\0';

        p2c.close(Pipe::PipeEnd::READ);
        c2p.close(Pipe::PipeEnd::WRITE);

        p2c.write(content.data(), content.size());
        p2c.close(Pipe::PipeEnd::WRITE);

        c2p.read(buffer, hash_length);
        c2p.close(Pipe::PipeEnd::READ);

        // wait for the child to finish and release its resources
        wait_wrap();

        return std::string(buffer);
    }

    // Function wrappers replace error codes with exceptions.

    static void pipe_wrap(int* pipedes)
    {
        if (pipe(pipedes) != 0)
            throw std::runtime_error("Failed pipe() call");
    }

    static pid_t fork_wrap()
    {
        pid_t pid = fork();
        if (pid == -1)
            throw std::runtime_error("Failed fork() call");
        return pid;
    }

    static void close_wrap(int fd)
    {
        if (close(fd) != 0)
            throw std::runtime_error("Failed close() call");
    }

    static void dup2_wrap(int old_fd, int new_fd)
    {
        if (dup2(old_fd, new_fd) == -1)
            throw std::runtime_error("Failed dup2() call");
    }

    static ssize_t write_wrap(int fd, const void* buf, ssize_t nbytes)
    {
        ssize_t n = write(fd, buf, nbytes);
        if (n == -1)
            throw std::runtime_error("Failed write() call");
        return n;
    }

    static ssize_t read_wrap(int fd, void* buf, size_t nbytes)
    {
        ssize_t n = read(fd, buf, nbytes);
        if (n == -1)
            throw std::runtime_error("Failed read() call");
        return n;
    }

    static void wait_wrap()
    {
        if (wait(nullptr) == -1)
            throw std::runtime_error("Failed wait() call");
    }

    static void exec_wrap(const char* prog)
    {
        if (execlp(prog, prog, nullptr) == -1)
            throw std::runtime_error("Failed execlp() call");
    }
};

#endif /* SRC_SHA256IDGENERATOR_HPP_ */
