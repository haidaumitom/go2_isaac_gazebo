/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#include "observation_buffer.hpp"

#include <cassert>
#include <iostream>
#include <vector>

void expect_equal(const std::vector<float>& actual, const std::vector<float>& expected)
{
    assert(actual == expected);
}

void test_time_priority()
{
    ObservationBuffer buffer(1, {2, 1}, 3, "time");
    buffer.insert({1, 2, 3});
    buffer.insert({4, 5, 6});
    buffer.insert({7, 8, 9});
    expect_equal(buffer.get_obs_vec({2, 1, 0}), {1, 2, 3, 4, 5, 6, 7, 8, 9});
}

void test_term_priority()
{
    ObservationBuffer buffer(1, {2, 1}, 3, "term");
    buffer.insert({1, 2, 3});
    buffer.insert({4, 5, 6});
    buffer.insert({7, 8, 9});
    expect_equal(buffer.get_obs_vec({2, 1, 0}), {1, 2, 4, 5, 7, 8, 3, 6, 9});
}

void test_warm_start()
{
    ObservationBuffer buffer(1, {3}, 3, "time");
    buffer.reset({0}, {1, 2, 3});
    expect_equal(buffer.get_obs_vec({2, 1, 0}), {1, 2, 3, 1, 2, 3, 1, 2, 3});
}

void test_thirty_frame_history()
{
    ObservationBuffer buffer(1, {1}, 30, "time");
    for (int value = 1; value <= 30; ++value)
    {
        buffer.insert({static_cast<float>(value)});
    }

    std::vector<int> oldest_to_newest;
    std::vector<float> expected;
    for (int index = 29; index >= 0; --index)
    {
        oldest_to_newest.push_back(index);
        expected.push_back(static_cast<float>(30 - index));
    }
    expect_equal(buffer.get_obs_vec(oldest_to_newest), expected);
}

int main()
{
    test_time_priority();
    test_term_priority();
    test_warm_start();
    test_thirty_frame_history();
    std::cout << "ObservationBuffer tests passed" << std::endl;
    return 0;
}
