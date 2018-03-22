#pragma once

#include <mutex>
#include <ostream>
#include <sstream>

/**
 * @brief Convinience class that enables thread safe output streams. The output is writen to the
 * target output stream, when the object is destroyed.
 */
class ThreadSafePrinter : public std::stringstream {
public:
    /**
     * @brief Creates a new ThreadSafePrinter.
     * @param out Output stream
     */
    ThreadSafePrinter(std::ostream &out);

    /**
     * @brief Destroys the ThreadSafePrinter.
     */
    virtual ~ThreadSafePrinter();

private:
    /**
     * @brief Output stream
     */
    std::ostream &m_out;

    /**
     * @brief Single mutex that guards all output streams
     */
    static std::mutex printMutex;
};

#define printOut ThreadSafePrinter(std::cout)
#define printErr ThreadSafePrinter(std::cerr)
