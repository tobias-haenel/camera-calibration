#include "ThreadSafePrinter.h"

using namespace std;

mutex ThreadSafePrinter::printMutex;

ThreadSafePrinter::ThreadSafePrinter(ostream &out) : m_out{out} {}

ThreadSafePrinter::~ThreadSafePrinter() {
    lock_guard<mutex> lock{printMutex};
    m_out << str() << std::flush;
}
