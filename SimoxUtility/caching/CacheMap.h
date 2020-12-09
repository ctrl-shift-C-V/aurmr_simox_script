#pragma once

#include <map>


#include <functional>


namespace simox::caching
{


    /**
     * @brief Wrapper for using a std::map or std::unordered_map as a data cache.
     */
    template <class KeyT, class ValueT, template <class ...> class MapT = std::map>
    class CacheMap
    {
    public:

        /// A fetch function which creates an item when its not already in the cache.
        using FetchFn = std::function<ValueT(KeyT key)>;


    public:

        CacheMap()
        {}

        CacheMap(FetchFn fetchFn) : fetchFn(fetchFn)
        {}


        void setFetchFn(FetchFn fetchFn, bool clear = false)
        {
            this->fetchFn = fetchFn;
            if (clear)
            {
                this->clear();
            }
        }


        ValueT& get(const KeyT& key, std::function<ValueT(KeyT)> fetchFn)
        {
            auto it = map.find(key);
            if (it != map.end())
            {
                return it->second;
            }
            else
            {
                return this->insert(key, fetchFn(key));
            }
        }

        ValueT& get(const KeyT& key)
        {
            return this->get(key, this->fetchFn);
        }


        // Explicit insert

        ValueT& insert(const KeyT& key, const ValueT& value)
        {
            return map.emplace(key, value).first->second;
        }
        ValueT& insert(const KeyT& key, std::function<ValueT(KeyT)> fetchFn)
        {
            return this->insert(key, fetchFn(key));
        }


        // Standard container functions

        void clear()
        {
            map.clear();
        }
        size_t size() const
        {
            return map.size();
        }
        bool empty() const
        {
            return map.empty();
        }
        bool contains(const KeyT& key) const
        {
            return map.count(key) > 0;
        }


    private:

        MapT<KeyT, ValueT> map;
        std::function<ValueT(KeyT)> fetchFn;

    };

}




