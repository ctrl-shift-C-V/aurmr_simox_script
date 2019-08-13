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
     * @brief Write a JSON document to the given file. 
     * 
     * @param j The JSON document.
     * @param filename The name of the file to write to.
     * @param indent See nlohmann::json::dump().
     * @param indent_char See nlohmann::json::dump().
     * 
     * @throw std::ios_base::failure If IO access fails.
     */
    void write_json(const std::string& filename, const json& j,
                    const int indent = -1, const char indent_char = ' ');


}
