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
 * @package    
 * @author     Fabian Reister ( fabian dot reister at kit dot edu )
 * @date       2021
 * @copyright  http://www.gnu.org/licenses/gpl-2.0.txt
 *             GNU General Public License
 */

#include <algorithm>
#include <boost/test/unit_test_suite.hpp>
#include <numeric>
#define BOOST_TEST_MODULE SimoxUtility/iterator/XYIndexRangeIterator

#include <boost/test/included/unit_test.hpp>

#include <vector>
#include <iostream>

#include <SimoxUtility/iterator.h>


BOOST_AUTO_TEST_SUITE(XYIndexRangeIterator)


BOOST_AUTO_TEST_CASE(test_loop)
{
    // dimensions
    constexpr int N = 10;
    constexpr int M = 20;

    // create 2D vector of size N*M
    const auto v = std::vector<std::vector<int>>(N, std::vector<int>(M, 1));

    const auto range_it = simox::iterator::XYIndexRangeIterator(v);
    
    // max counters for test checks
    int x_max = 0;
    int y_max = 0;
    int cnt = 0;

    for(const auto [x,y] : range_it){
        x_max = std::max(x, x_max);
        y_max = std::max(x, y_max);

        cnt++;
    }

    BOOST_CHECK_EQUAL(x_max, N-1);
    BOOST_CHECK_EQUAL(y_max, M-1);
    BOOST_CHECK_EQUAL(cnt, N*M);
}


BOOST_AUTO_TEST_CASE(test_foreach)
{
    // dimensions
    constexpr int N = 10;
    constexpr int M = 20;

    // create 2D vector of size N*M
    const auto v = std::vector<std::vector<int>>(N, std::vector<int>(M, 1));

    const auto range_it = simox::iterator::XYIndexRangeIterator(v);
    
    int x_max = 0;
    int y_max = 0;
    int cnt = 0;

    std::for_each(range_it.begin(), range_it.end(), [&](const auto& pnt){
        const auto [x,y] = pnt;

        x_max = std::max(x, x_max);
        y_max = std::max(x, y_max);

        cnt++;
    });

    BOOST_CHECK_EQUAL(x_max, N-1);
    BOOST_CHECK_EQUAL(y_max, M-1);
    BOOST_CHECK_EQUAL(cnt, N*M);
}

BOOST_AUTO_TEST_SUITE_END()
