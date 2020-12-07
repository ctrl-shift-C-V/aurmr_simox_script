#pragma once

#include <condition_variable>
#include <mutex>


namespace simox::threads
{

    /**
     * @brief A counting semaphore.
     *
     * Threads can enter when the internal count is > 0.
     * Notifiying the semaphore increments the count and allows threads to enter.
     * A thread can wait until it may enter. When it enters, it decrements
     * the internal count.
     *
     * Can be used e.g. in a Producer-Consumer pattern.
     * The producer signals new jobs via `notify()`, while the consumer waits
     * for new jobs via `wait()`.
     */
    class CountingSemaphore
    {
    public:

        /// Construct an initially blocking semaphore (initial count 0).
        CountingSemaphore();
        /// Construct a semaphore with the given count.
        CountingSemaphore(unsigned int count);

        /**
         * @brief Signal that one waiting thread may continue.
         * Also known as `post()` or `signal()`.
         */
        void notify();

        /**
         * @brief Wait until a thread may enter.
         */
        void wait();

        /**
         * @brief Try to enter. If the semaphore is currently blocking, return false.
         * @return True if entering was successful, false if semaphore was blocking.
         */
        bool try_wait();


    private:

        /// The mutex for _condition and _count.
        std::mutex _mutex;
        /// The condition variable to wake up waking threads.
        std::condition_variable _condition;

        /// The current count. Waiting threads may enter when > 0.
        unsigned int _count = 0;

    };

}
