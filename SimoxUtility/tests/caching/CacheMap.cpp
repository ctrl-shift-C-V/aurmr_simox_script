/*
 * This file is part of ArmarX.
 *
 * ArmarX is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * ArmarX is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * @package    RobotAPI::ArmarXObjects::FrameTracking
 * @author     Adrian Knobloch ( adrian dot knobloch at student dot kit dot edu )
 * @date       2019
 * @copyright  http://www.gnu.org/licenses/gpl-2.0.txt
 *             GNU General Public License
 */

#define BOOST_TEST_MODULE SimoxUtility/color/ColorMap

#include <unordered_map>

#include <boost/test/included/unit_test.hpp>

#include <SimoxUtility/caching/CacheMap.h>

#include <iostream>


namespace CacheMapTest
{

}


BOOST_AUTO_TEST_SUITE(CacheMapTest)


BOOST_AUTO_TEST_CASE(test_at_empty)
{
    auto fetchFn = [](int i)
    {
        return std::to_string(i);
    };

    simox::caching::CacheMap<int, std::string> cache(fetchFn);

    BOOST_CHECK(cache.empty());
    BOOST_CHECK_EQUAL(cache.size(), 0);

    BOOST_CHECK_EQUAL(cache.get(1), "1");
    BOOST_CHECK_EQUAL(cache.size(), 1);

    BOOST_CHECK_EQUAL(cache.get(1), "1");
    BOOST_CHECK_EQUAL(cache.size(), 1);

    BOOST_CHECK_EQUAL(cache.get(2), "2");
    BOOST_CHECK_EQUAL(cache.size(), 2);

    BOOST_CHECK_EQUAL(cache.get(2), "2");
    BOOST_CHECK_EQUAL(cache.size(), 2);


    const bool clear = true;
    cache.setFetchFn([](int i)
    {
        return std::to_string(-i);
    }, clear);
    BOOST_CHECK_EQUAL(cache.size(), 0);
    BOOST_CHECK(cache.empty());
}



BOOST_AUTO_TEST_SUITE_END()
