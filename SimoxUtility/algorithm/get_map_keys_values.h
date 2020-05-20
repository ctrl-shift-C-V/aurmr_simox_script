#include <map>
#include <vector>


namespace simox
{


    template <typename K, typename V>
    std::vector<K> get_keys(const std::map<K, V>& map)
    {
        std::vector<K> keys;
        keys.reserve(map.size());
        for (const auto& [k, v] : map)
        {
            keys.push_back(k);
        }
        return keys;
    }


    template <typename K, typename V>
    std::vector<V> get_values(const std::map<K, V>& map)
    {
        std::vector<V> values;
        values.reserve(map.size());
        for (const auto& [k, v] : map)
        {
            values.push_back(v);
        }
        return values;
    }


}
