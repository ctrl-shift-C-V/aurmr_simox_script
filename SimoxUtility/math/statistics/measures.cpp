#include "measures.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <SimoxUtility/error/SimoxError.h>


static void checkNotEmpty(const std::vector<float>& values)
{
    if (values.empty())
    {
        throw simox::error::SimoxError("Passed vector of values is empty.");
    }
}

void simox::math::sort(std::vector<float>& values)
{
    std::sort(values.begin(), values.end());
}

std::vector<float> simox::math::sorted(const std::vector<float>& values)
{
    std::vector<float> s = values;
    std::sort(s.begin(), s.end());
    return s;
}

float simox::math::min(const std::vector<float>& values, bool isSorted)
{
    checkNotEmpty(values);
    return isSorted ? values.front() : *std::min_element(values.begin(), values.end());
}

float simox::math::max(const std::vector<float>& values, bool isSorted)
{
    checkNotEmpty(values);
    return isSorted ? values.back() : *std::max_element(values.begin(), values.end());
}

float simox::math::mean(const std::vector<float>& values)
{
    checkNotEmpty(values);

    float sum = 0;
    for (float v : values)
    {
        sum += v;
    }
    return sum / values.size();
}

float simox::math::stddev(const std::vector<float>& values)
{
    return stddev(values, mean(values));
}

float simox::math::stddev(const std::vector<float>& values, float mean)
{
    checkNotEmpty(values);
    float sum = 0;
    for (float v : values)
    {
        float diff = v - mean;
        sum += diff * diff;
    }
    float variance = sum / (values.size() - 1);
    return std::sqrt(variance);
}

float simox::math::quantile(const std::vector<float>& _values, float p, bool isSorted)
{
    checkNotEmpty(_values);
    const std::vector<float>& values = isSorted ? _values : sorted(_values);

    float location = p < 1 ? p * values.size() : values.size() - 1;

    std::size_t floor = static_cast<std::size_t>(std::floor(location));
    std::size_t ceil = static_cast<std::size_t>(std::ceil(location));

    if (floor == ceil)
    {
        return values.at(floor);
    }
    else
    {
        float t = location - floor;
        return (1 - t) * values.at(floor) + t * values.at(ceil);
    }
}

float simox::math::lower_quartile(const std::vector<float>& values, bool isSorted)
{
    return quantile(values, .25, isSorted);
}

float simox::math::median(const std::vector<float>& values, bool isSorted)
{
    return quantile(values, .5, isSorted);
}

float simox::math::upper_quartile(const std::vector<float>& values, bool isSorted)
{
    return quantile(values, .75, isSorted);
}

float simox::math::interquartile_range(const std::vector<float>& _values, bool isSorted)
{
    checkNotEmpty(_values);

    const std::vector<float>& values = isSorted ? _values : sorted(_values);
    return interquartile_range(lower_quartile(values, true), upper_quartile(values, true));
}

float simox::math::interquartile_range(float lowerQuartile, float upperQuartile)
{
    return upperQuartile - lowerQuartile;
}

