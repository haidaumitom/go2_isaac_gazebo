/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#include "observation_buffer.hpp"
#include <stdexcept>
#include <algorithm>

ObservationBuffer::ObservationBuffer()
{
}

ObservationBuffer::ObservationBuffer(int num_envs,
                                     const std::vector<int>& obs_dims,
                                     int history_length,
                                     const std::string& priority)
    : num_envs(num_envs),
      obs_dims(obs_dims),
      priority(priority),
      history_length(history_length)
{
    if (num_envs <= 0 || history_length <= 0)
    {
        throw std::invalid_argument("num_envs and history_length must be positive");
    }
    if (priority != "time" && priority != "term")
    {
        throw std::invalid_argument("Observation priority must be 'time' or 'term'");
    }

    for (int dim : obs_dims)
    {
        if (dim <= 0)
        {
            throw std::invalid_argument("All observation dimensions must be positive");
        }
        num_obs += dim;
    }

    num_obs_total = num_obs;
    if (num_obs_total <= 0)
    {
        throw std::runtime_error("Invalid total observation dimension");
    }

    // Initialize buffer: [env][time][obs]
    obs_buf.resize(num_envs);
    for (int env_idx = 0; env_idx < num_envs; ++env_idx)
    {
        obs_buf[env_idx].resize(history_length);
        for (int t = 0; t < history_length; ++t)
        {
            obs_buf[env_idx][t].resize(num_obs_total, 0.0f);
        }
    }
}

void ObservationBuffer::reset(std::vector<int> reset_idxs, const std::vector<float>& new_obs)
{
    if (obs_buf.empty())
    {
        return;
    }
    if (new_obs.size() != static_cast<size_t>(num_obs_total))
    {
        throw std::invalid_argument("Reset observation size does not match the configured observation dimension");
    }

    // Reset observation buffer for specified environments
    for (int env_idx : reset_idxs)
    {
        if (env_idx >= 0 && env_idx < num_envs)
        {
            // Copy new observation data to all time steps
            for (int t = 0; t < history_length; ++t)
            {
                for (int i = 0; i < num_obs_total && i < static_cast<int>(new_obs.size()); ++i)
                {
                    obs_buf[env_idx][t][i] = new_obs[i];
                }
            }
        }
    }
}

void ObservationBuffer::insert(const std::vector<float>& new_obs)
{
    if (obs_buf.empty())
    {
        return;
    }
    if (new_obs.size() != static_cast<size_t>(num_obs_total))
    {
        throw std::invalid_argument("Inserted observation size does not match the configured observation dimension");
    }

    // Shift historical observations forward by one position for all environments
    for (int env_idx = 0; env_idx < num_envs; ++env_idx)
    {
        // Move from back to front to avoid overwriting
        for (int t = history_length - 1; t > 0; --t)
        {
            obs_buf[env_idx][t] = obs_buf[env_idx][t - 1];
        }

        // Insert new observation at the first position
        obs_buf[env_idx][0] = new_obs;
    }
}

/**
 * @brief Gets history of observations indexed by obs_ids.
 *
 * @param obs_ids An array of integers with which to index the desired
 *                observations, where 0 is the latest observation and
 *                history_length - 1 is the oldest observation.
 * @return A vector containing the concatenated observations.
 */
std::vector<float> ObservationBuffer::get_obs_vec(std::vector<int> obs_ids)
{
    if (obs_buf.empty() || obs_ids.empty())
    {
        return std::vector<float>();
    }

    int valid_history_frames = 0;
    for (int obs_id : obs_ids)
    {
        if (obs_id < 0 || obs_id >= history_length)
        {
            throw std::out_of_range("Observation history index is outside the configured buffer");
        }
        ++valid_history_frames;
    }

    // Create output vector
    std::vector<float> output;
    output.reserve(num_envs * valid_history_frames * num_obs_total);

    if (this->priority == "time")
    {
        // Time priority: iterate environments first, then time steps, finally observation dimensions
        for (int env_idx = 0; env_idx < num_envs; ++env_idx)
        {
            for (int obs_id : obs_ids)
            {
                // obs_id=0 is newest and obs_id=N is N control steps old.
                int slice_idx = obs_id;
                for (int i = 0; i < num_obs_total; ++i)
                {
                    output.push_back(obs_buf[env_idx][slice_idx][i]);
                }
            }
        }
    }
    else if (this->priority == "term")
    {
        // Term priority: iterate environments first, then observation terms, finally time steps
        for (int env_idx = 0; env_idx < num_envs; ++env_idx)
        {
            int obs_offset = 0;
            for (size_t i = 0; i < obs_dims.size(); ++i)
            {
                int dim = obs_dims[i];
                for (int step : obs_ids)
                {
                    int time_offset = step;
                    for (int j = 0; j < dim; ++j)
                    {
                        output.push_back(obs_buf[env_idx][time_offset][obs_offset + j]);
                    }
                }
                obs_offset += dim;
            }
        }
    }

    return output;
}
