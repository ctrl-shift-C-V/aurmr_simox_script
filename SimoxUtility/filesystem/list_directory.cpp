#include "list_directory.h"

#include <algorithm>


namespace simox
{
    std::vector<fs::path> fs::list_directory(const path& directory, bool local, bool sort)
    {
        std::vector<path> entries;
        for (const auto& entry : directory_iterator(directory))
        {
            entries.push_back(entry.path());
        }

        if (sort)
        {
            std::sort(entries.begin(), entries.end());
        }

        if (local)
        {
            for (auto& entry : entries)
            {
                entry = entry.filename();
            }
        }
        return entries;
    }

}

