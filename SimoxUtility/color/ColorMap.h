#pragma once

#include <map>

#include "Color.h"
#include "interpolation.h"


namespace simox::color
{

    class ColorMap
    {
    public:

        ColorMap();
        ColorMap(std::initializer_list<Color> init);
        ColorMap(std::initializer_list<std::pair<float, Color>> init);

        bool empty() const;
        size_t size() const;

        void clear();
        void addKey(float value, const Color& color);

        Color at(float value) const;
        inline Color operator()(float value) const { return this->at(value); }


    private:

        /// Map of value to color at that value.
        std::map<float, Color> keys;


    };

}
