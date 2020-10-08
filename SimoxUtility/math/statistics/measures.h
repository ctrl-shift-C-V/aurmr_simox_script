#pragma once

#include <vector>


namespace simox::math
{

    void sort(std::vector<float>& values);
    std::vector<float> sorted(const std::vector<float>& values);


    float min(const std::vector<float>& values, bool isSorted=false);
    float max(const std::vector<float>& values, bool isSorted=false);
    float mean(const std::vector<float>& values);

    float stddev(const std::vector<float>& values);
    float stddev(const std::vector<float>& values, float mean);

    float quantile(const std::vector<float>& values, float p, bool isSorted=false);

    float lower_quartile(const std::vector<float>& values, bool isSorted=false);
    float median(const std::vector<float>& values, bool isSorted=false);
    float upper_quartile(const std::vector<float>& values, bool isSorted=false);

    float interquartile_range(const std::vector<float>& values, bool isSorted=false);
    float interquartile_range(float lower_quartile, float upper_quartile);

}
