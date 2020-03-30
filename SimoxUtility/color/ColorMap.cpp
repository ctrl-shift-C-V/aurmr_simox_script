#include "ColorMap.h"


namespace simox::color
{

    ColorMap::ColorMap()
    {
    }

    ColorMap::ColorMap(std::initializer_list<Color> init)
    {
        if (init.size() == 1)
        {
            addKey(0, *init.begin());
        }
        else
        {
            size_t i = 0;
            float max = init.size() - 1;
            for (const auto& color : init)
            {
                addKey(i++ / max, color);
            }
        }
    }

    ColorMap::ColorMap(std::initializer_list<std::pair<float, Color> > init)
    {
        for (const auto& [v, c] : init)
        {
            addKey(v, c);
        }
    }


    bool ColorMap::empty() const
    {
        return keys.empty();
    }

    size_t ColorMap::size() const
    {
        return keys.size();
    }


    void ColorMap::clear()
    {
        keys.clear();
    }


    void ColorMap::addKey(float value, const Color& color)
    {
        keys[value] = color;
    }


    Color ColorMap::at(float value) const
    {
        if (empty())
        {
            return Color::black();
        }

        if (keys.size() == 1)
        {
            return keys.begin()->second;
        }

        // keys.size() >= 2

        if (value <= keys.begin()->first)
        {
            return keys.begin()->second;
        }

        // map::lower_bound(): lb not less than value (lb >= value)
        // map::upper_bound(): ub greater  than value (ub > value)
        auto upper = keys.upper_bound(value);
        if (upper == keys.end())
        {
            // No entry > value  =>  value >= all entries.
            --upper;
            return upper->second;
        }

        auto lower = upper;
        --lower;

        // Interpolate between lower and upper

        float t = (value - lower->first) / (upper->first - lower->first);
        // t = 0 => full lower,  t = 1 => full upper

        return interpol::linear(t, lower->second, upper->second);
    }

}
