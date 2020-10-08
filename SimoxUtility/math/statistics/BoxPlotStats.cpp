#include "BoxPlotStats.h"

#include "measures.h"


namespace simox::math
{


BoxPlotStats::BoxPlotStats() = default;


BoxPlotStats::BoxPlotStats(const std::vector<float>& values, bool isSorted, float whisk) :
    whisk(whisk)
{
    set(values, isSorted);
}

void BoxPlotStats::set(const std::vector<float>& _values, bool isSorted)
{
    const std::vector<float>& values = isSorted ? _values : math::sorted(_values);

    this->minimum = math::min(values, true);
    this->maximum = math::max(values, true);

    this->lowerQuartile = math::lower_quartile(values, true);
    this->median = math::median(values, true);
    this->upperQuartile = math::upper_quartile(values, true);

    const float iqr = math::interquartile_range(lowerQuartile, upperQuartile);

    this->minWhisker = lowerQuartile - whisk * iqr;
    this->maxWhisker = upperQuartile + whisk * iqr;

    // Compute outliers and correct whiskers if necessary.
    {
        auto it = values.begin();
        for (; it != values.end() && *it < minWhisker; ++it)
        {
            outliers.push_back(*it);
        }
        minWhisker = (it != values.begin()) ? *it : minimum;
    }
    {
        auto rit = values.rbegin();
        for (; rit != values.rend() && *rit > maxWhisker; ++rit)
        {
            outliers.push_back(*rit);
        }
        maxWhisker = (rit != values.rbegin()) ? *rit : maximum;
    }
}

}
