#include "SoftMinMax.h"

#include <cmath>
#include <stdexcept>
#include <sstream>


namespace simox::math
{

    SoftMinMax::SoftMinMax()
    {
        reset(0, 1);
    }

    SoftMinMax::SoftMinMax(float percentile, std::size_t numValues)
    {
        reset(percentile, numValues);
    }

    void SoftMinMax::reset(float percentile, std::size_t numValues)
    {
        min_queue = MinQueue();
        max_queue = MaxQueue();

        if (percentile < 0 || percentile > 0.5f)
        {
            std::stringstream msg;
            msg << "percentile must be in [0, 0.5], but was " << percentile << ".";
            throw std::invalid_argument(msg.str());
        }
        if (numValues == 0)
        {
            std::stringstream msg;
            msg << "numValues must be > 0, but was " << numValues;
            throw std::invalid_argument(msg.str());
        }
        this->percentile = percentile;
        this->num_elements = numValues;
    }

    void SoftMinMax::add(float value)
    {
        if (min_queue.size() != max_queue.size())
        {
            std::stringstream msg;
            msg << "Invariant violated: minQueue.size() == maxQueue.size(), "
                << "but were " << min_queue.size() << " and " << max_queue.size();
            throw std::logic_error(msg.str());
        }

        if (min_queue.size() < allowed_heap_size())
        {
            // Heaps not full yet
            min_queue.push(value);
            max_queue.push(value);
            return;
        }

        // Heaps are full

        // Check minQueue
        if (value < min_queue.top())
        {
            // insert and pop
            min_queue.push(value);
            min_queue.pop();
        } // else ignore value

        // Check maxQueue
        if (value > max_queue.top())
        {
            max_queue.push(value);
            max_queue.pop();
        }

    }

    float SoftMinMax::get_soft_min() const
    {
        if (max_queue.empty())
        {
            throw std::out_of_range("Calling getSoftMin() but no element was added.");
        }
        return min_queue.top();
    }

    float SoftMinMax::get_soft_max() const
    {
        if (max_queue.empty())
        {
            throw std::out_of_range("Calling getSoftMin() but no element was added.");
        }

        return max_queue.top();
    }

    std::size_t SoftMinMax::num_outside_soft_min_max() const
    {
        return size_t(std::ceil(percentile * num_elements));
    }

    std::size_t SoftMinMax::allowed_heap_size() const
    {
        return std::max(std::size_t(1), num_outside_soft_min_max() + 1);
    }

}
