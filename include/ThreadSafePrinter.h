#pragma once

#include <mutex>
#include <ostream>
#include <sstream>

class ThreadSafePrinter : public std::stringstream {
public:
    ThreadSafePrinter(std::ostream &out);

    ~ThreadSafePrinter();

private:
    std::ostream &m_out;

    static std::mutex printMutex;
};

#define printOut ThreadSafePrinter(std::cout)
#define printErr ThreadSafePrinter(std::cerr)
