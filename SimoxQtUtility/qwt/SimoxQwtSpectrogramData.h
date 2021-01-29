#pragma once

#include <deque>
#include <complex>
#include <vector>

#include <qwt_plot_spectrogram.h>

namespace simox::qt
{
    class SimoxQwtSpectrogramData: public QwtRasterData
    {
    public:
        SimoxQwtSpectrogramData(std::size_t iterations, std::size_t max_fft_width);

        virtual double value(double fx, double fy) const;
        
        const std::vector<float>& update(const std::vector<std::complex<float>>& v, std::size_t n);
        const std::vector<float>& update(const std::vector<std::complex<float>>& v);
        const std::vector<float>& update(std::vector<float> v);
        std::size_t iterations() const;
        void setIterations(std::size_t iterations);
    private:
        std::deque<std::vector<float>> _vals;
        std::size_t                    _iterations;
    };
}
