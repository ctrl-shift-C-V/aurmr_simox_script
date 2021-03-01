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
 * @package    SimoxUtility/Iterators
 * @author     Fabian Reister ( fabian dot reister at kit dot edu )
 * @date       2019
 * @copyright  http://www.gnu.org/licenses/gpl-2.0.txt
 *             GNU General Public License
 */

#include <algorithm>
#include <numeric>
#define BOOST_TEST_MODULE SimoxUtility/iterator/NestedRangeIterator

#include <boost/test/included/unit_test.hpp>

#include <vector>
#include <iostream>

#include <SimoxUtility/iterator.h>


// namespace NestedRangeIterator
// {

// }


BOOST_AUTO_TEST_SUITE(NestedRangeIterator)


BOOST_AUTO_TEST_CASE(test_loop)
{
    // dimensions
    constexpr int N = 10;
    constexpr int M = 20;

    // create 2D vector of size N*M
    auto v = std::vector<std::vector<int>>(N, std::vector<int>(M, 1));

    auto it = simox::iterator::NestedRangeIterator(v);  
    const int sum = std::accumulate(it.begin(), it.end(), 0);

    // std::for_each(it.begin(), it.end(), [](auto i){});

    //int sum = 200;
    BOOST_CHECK_EQUAL(sum, N*M);

    std::cout << "finished";

}

BOOST_AUTO_TEST_SUITE_END()
