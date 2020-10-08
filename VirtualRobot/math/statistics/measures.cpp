#include "measures.h"

#include <SimoxUtility/math/statistics/measures.h>


namespace math { namespace stat
{

void sort(std::vector<float>& values)
{
    simox::math::sort(values);
}

std::vector<float> sorted(const std::vector<float>& values)
{
    return simox::math::sorted(values);
}

float min(const std::vector<float>& values, bool isSorted)
{
    return simox::math::min(values, isSorted);
}

float max(const std::vector<float>& values, bool isSorted)
{
    return simox::math::max(values, isSorted);
}

float mean(const std::vector<float>& values)
{
    return simox::math::mean(values);
}

float stddev(const std::vector<float>& values)
{
    return simox::math::stddev(values);
}

float stddev(const std::vector<float>& values, float mean)
{
    return simox::math::stddev(values, mean);
}

float quantile(const std::vector<float>& values, float p, bool isSorted)
{
    return simox::math::quantile(values, p, isSorted);
}

float lowerQuartile(const std::vector<float>& values, bool isSorted)
{
    return simox::math::lower_quartile(values, isSorted);
}

float median(const std::vector<float>& values, bool isSorted)
{
    return simox::math::median(values, isSorted);
}

float upperQuartile(const std::vector<float>& values, bool isSorted)
{
    return simox::math::upper_quartile(values, isSorted);
}

float interquartileRange(const std::vector<float>& values, bool isSorted)
{
    return simox::math::interquartile_range(values, isSorted);
}

float interquartileRange(float lowerQuartile, float upperQuartile)
{
    return simox::math::interquartile_range(lowerQuartile, upperQuartile);
}


}}
