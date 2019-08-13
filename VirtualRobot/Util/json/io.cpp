#include "io.h"

#include <iostream>
#include <fstream>


namespace nlohmann
{

    json read_json(const std::string& filename)
    {
        std::ifstream ifs;
        // Allow throwing std::ios_base::failure.
        ifs.exceptions(std::ifstream::failbit | std::ifstream::badbit);
        
        ifs.open(filename);
        json j;
        ifs >> j;
        return j;
    }
    
    void write_json(const std::string& filename, const json& j,
                    const int indent, const char indent_char)
    {
        std::ofstream ofs;
        // Allow throwing std::ios_base::failure.
        ofs.exceptions(std::ifstream::failbit | std::ifstream::badbit);
        
        ofs.open(filename);
        ofs << j.dump(indent, indent_char);
    }

}


