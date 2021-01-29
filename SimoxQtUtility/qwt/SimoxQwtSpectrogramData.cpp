#include "SimoxQwtSpectrogramData.h"

namespace simox::qt
{
    SimoxQwtSpectrogramData::SimoxQwtSpectrogramData(std::size_t iterations, std::size_t max_fft_width) :
        _iterations{iterations}
    {
        setInterval(Qt::XAxis, QwtInterval(0, iterations));
        setInterval(Qt::YAxis, QwtInterval(0, max_fft_width));
        setInterval(Qt::ZAxis, QwtInterval(0.0, 10.0));
    }

    double SimoxQwtSpectrogramData::value(double fx, double fy) const
    {
        const std::size_t x = fx;
        const std::size_t y = fy;
        if (x >= _vals.size())
        {
            return 0;
        }
        if (y >= _vals.at(x).size())
        {
            return 0;
        }
        return _vals.at(x).at(y);
    }

    const std::vector<float>& SimoxQwtSpectrogramData::update(
        const std::vector<std::complex<float>>& v)
    {
        return update(v, v.size());
    }

    const std::vector<float>& SimoxQwtSpectrogramData::update(
        const std::vector<std::complex<float>>& v,
        std::size_t n)
    {
        std::vector<float> init(n);
        for (std::size_t i = 0; i < n; ++i)
        {
            init.at(i) = std::abs(v.at(i));
        }
        return update(std::move(init));
    }

    const std::vector<float>& SimoxQwtSpectrogramData::update(std::vector<float> v)
    {
        while (_vals.size() < _iterations)
        {
            _vals.emplace_front();
        }
        _vals.emplace_back(std::move(v));
        while (_vals.size() > _iterations)
        {
            _vals.pop_front();
        }
        return _vals.back();
    }

    std::size_t SimoxQwtSpectrogramData::iterations() const
    {
        return _iterations;
    }

    void SimoxQwtSpectrogramData::setIterations(std::size_t iterations)
    {
        _iterations = iterations;
    }
}
