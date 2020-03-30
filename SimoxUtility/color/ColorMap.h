#pragma once

#include <map>

#include "Color.h"
#include "interpolation.h"


namespace simox::color
{

    /**
     * @brief A color map, mapping scalar values to colors.
     */
    class ColorMap
    {
    public:

        /// Construct an empty color map. Will always return black.
        ColorMap();

        /// Construct an unnamed color map with equidistant values from 0 to 1 (inclusive).
        ColorMap(std::initializer_list<Color> init);
        /// Construct an unnamed color map with given values.
        ColorMap(std::initializer_list<std::pair<float, Color>> init);

        /// Construct a named color map with equidistant values from 0 to 1 (inclusive).
        ColorMap(const std::string& name, std::initializer_list<Color> init);
        /// Construct a named color map with given values.
        ColorMap(const std::string& name, std::initializer_list<std::pair<float, Color>> init);


        bool empty() const;
        /// Get the number of keys.
        size_t size() const;

        void clear();
        /// Add a key color at the given value.
        void addKey(float value, const Color& color);

        /**
         * @brief Get the color for the given scalar value.
         *
         * Empty color maps always return black.
         *
         * @param value The scalar value.
         * @return
         */
        Color at(float value) const;

        /// @see `ColorMap::at()`
        inline Color operator()(float value) const { return this->at(value); }

        std::string name() const { return _name; }
        void setName(const std::string& name) { this->_name = name; }


    private:


        /// Map of value to color at that value.
        std::map<float, Color> keys;

        /// The name.
        std::string _name = "";

    };

}


namespace simox
{
    using ColorMap = color::ColorMap;
}
