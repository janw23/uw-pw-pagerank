#ifndef SRC_SHA256IDGENERATOR_HPP_
#define SRC_SHA256IDGENERATOR_HPP_

#include <unistd.h>
#include <sys/wait.h>

#include "immutable/idGenerator.hpp"
#include "immutable/pageId.hpp"

class Sha256IdGenerator : public IdGenerator {
public:
    virtual PageId generateId(std::string const &content) const {
        // todo Implement pipes as RAII based on what I have here.

        // pipe file descriptors
        int parent_to_child_pipe[2];
        int child_to_parent_pipe[2];

        // create pipes
        pipe_wrap(parent_to_child_pipe);
        pipe_wrap(child_to_parent_pipe);

        pid_t pid = fork_wrap();

        if (pid == 0) {
            exec_child_path(parent_to_child_pipe, child_to_parent_pipe);
            throw std::runtime_error("exec_child_path() neither threw nor exited");
        } else {
            std::string hash = exec_parent_path(
                    parent_to_child_pipe, child_to_parent_pipe, content);
            return PageId(std::move(hash));
        }
    }

private:
    void exec_child_path(int *p2c, int *c2p) const {
        const int READ = 0, WRITE = 1;
        // close unused pipe ends
        close_wrap(p2c[WRITE]);
        close_wrap(c2p[READ]);

        // swap std streams with pipe ends
        dup2_wrap(p2c[READ], STDIN_FILENO);
        dup2_wrap(c2p[WRITE], STDOUT_FILENO);

        // close pipe ends because they were duplicated
        close_wrap(p2c[READ]);
        close_wrap(c2p[WRITE]);

        exec_wrap("sha256sum");
    }

    std::string exec_parent_path(int *p2c, int *c2p, std::string const &content) const {
        const int READ = 0, WRITE = 1;

        const uint hash_length = 64; // sha256 hash has fixed character length
        char buffer[hash_length + 1]; // todo Is it okay? Maybe put it as class member
        buffer[hash_length] = '\0';

        // close unused pipe ends
        close_wrap(p2c[READ]);
        close_wrap(c2p[WRITE]);

        // send data to child process
        write_wrap(p2c[WRITE], content.data(), content.size());
        close_wrap(p2c[WRITE]);

        // receive result from child process
        read_wrap(c2p[READ], buffer, hash_length);
        close_wrap(c2p[READ]);

        // wait for the child to finish and release its resources
        wait_wrap();

        return std::string(buffer);
    }

    // Function wrappers replace error codes with exceptions.

    static void pipe_wrap(int *pipedes) {
        if (pipe(pipedes) != 0)
            throw std::runtime_error("Failed pipe() call");
    }

    static pid_t fork_wrap() {
        pid_t pid = fork();
        if (pid == -1)
            throw std::runtime_error("Failed fork() call");
        return pid;
    }

    static void close_wrap(int fd) {
        if (close(fd) != 0)
            throw std::runtime_error("Failed close() call");
    }

    static void dup2_wrap(int old_fd, int new_fd) {
        if (dup2(old_fd, new_fd) == -1)
            throw std::runtime_error("Failed dup2() call");
    }

    static ssize_t write_wrap(int fd, const void *buf, ssize_t nbytes) {
        ssize_t n = write(fd, buf, nbytes);
        if (n == -1)
            throw std::runtime_error("Failed write() call");
        return n;
    }

    static ssize_t read_wrap(int fd, void *buf, size_t nbytes) {
        ssize_t n = read(fd, buf, nbytes);
        if (n == -1)
            throw std::runtime_error("Failed read() call");
        return n;
    }

    static void wait_wrap() {
        if (wait(nullptr) == -1)
            throw std::runtime_error("Failed wait() call");
    }

    static void exec_wrap(const char *prog) {
        if (execlp(prog, prog, nullptr) == -1)
            throw std::runtime_error("Failed execlp() call");
    }

};

#endif /* SRC_SHA256IDGENERATOR_HPP_ */
