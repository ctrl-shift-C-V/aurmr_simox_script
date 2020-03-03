#pragma once

#include "json.hpp"


namespace nlohmann
{

    /**
     * @brief Read a JSON document from the given file.
     *
     * @param filename The name of the file to read from.
     * @return The JSON document.
     *
     * @throw std::ios_base::failure If IO access fails.
     */
    json read_json(const std::string& filename);

    /**
     * @brief Read a JSON document from the given in-stream.
     * @param is The in-stream.
     * @return The JSON document.
     */
    json read_json(std::istream& is);


    /**
     * @brief Write a JSON document to the given file.
     *
     * @param filename The name of the file to write to.
     * @param j The JSON document.
     * @param indent See nlohmann::json::dump().
     * @param indent_char See nlohmann::json::dump().
     *
     * @throw std::ios_base::failure If IO access fails.
     */
    void write_json(const std::string& filename, const json& j,
                    const int indent = -1, const char indent_char = ' ');

    /**
     * @brief Write a JSON document to the given out-stream.
     * @param os The out-stream.
     * @param j The JSON document.
     * @param indent See nlohmann::json::dump().
     * @param indent_char See nlohmann::json::dump().
     */
    void write_json(std::ostream& os, const json& j,
                    const int indent = -1, const char indent_char = ' ');

}
