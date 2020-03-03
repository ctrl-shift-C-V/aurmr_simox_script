#include "io.h"

#include <iostream>
#include <filesystem>
#include <fstream>


namespace fs = std::filesystem;


namespace
{
    void checkExists(const std::string& filename, const std::string& prefix = "")
    {
        if (!fs::exists(filename))
        {
            throw std::ios_base::failure(prefix + "File \"" + filename + "\" does not exist.");
        }
    }
}

namespace nlohmann
{
    json read_json(const std::string& filename)
    {
        std::ifstream ifs;
        // Allow throwing std::ios_base::failure.
        ifs.exceptions(std::ifstream::failbit | std::ifstream::badbit);

        try
        {
            ifs.open(filename);
            return read_json(ifs);
        }
        catch (const std::ios_base::failure& e)
        {
            // Add a more useful message.
            const std::string msg = "Failed to read file \"" + filename + "\": ";
            checkExists(filename, msg);
            throw std::ios_base::failure(msg + std::string(e.what()));
        }
    }


    json read_json(std::istream& is)
    {
        json j;
        is >> j;
        return j;
    }


    void write_json(const std::string& filename, const json& j,
                    const int indent, const char indent_char)
    {
        std::ofstream ofs;
        // Allow throwing std::ios_base::failure.
        ofs.exceptions(std::ifstream::failbit | std::ifstream::badbit);

        try
        {
            ofs.open(filename);
            write_json(ofs, j, indent, indent_char);
        }
        catch (const std::ios_base::failure& e)
        {
            // Add a more useful message.
            const std::string msg = "Failed to write file \"" + filename + "\": ";
            throw std::ios_base::failure(msg + std::string(e.what()));
        }
    }


    void write_json(std::ostream& os, const json& j, const int indent, const char indent_char)
    {
        os << j.dump(indent, indent_char);
    }

}


