#pragma once

#include <algorithm>
#include <random>
#include <numeric>
#include <cmath>
#include <omp.h>

namespace dogm {

template<typename T>
inline T clamp(T val, T min_val, T max_val) {
    return std::max(min_val, std::min(val, max_val));
}

inline unsigned int hash(unsigned int a) {
    a = (a + 0x7ed55d16) + (a << 12);
    a = (a ^ 0xc761c23c) ^ (a >> 19);
    a = (a + 0x165667b1) + (a << 5);
    a = (a + 0xd3a2646c) ^ (a << 9);
    a = (a + 0xfd7046c5) + (a << 3);
    a = (a ^ 0xb55a4f09) ^ (a >> 16);
    return a;
}

template<typename T>
void accumulate(const std::vector<T>& input, std::vector<T>& output) {
    output.resize(input.size());
    std::partial_sum(input.begin(), input.end(), output.begin());
}

template<typename T>
inline T subtract(const std::vector<T>& accum_array, int start_idx, int end_idx) {
    if (start_idx == 0) {
        return accum_array[end_idx];
    }
    return accum_array[end_idx] - accum_array[start_idx - 1];
}

class RandomGenerator {
private:
    std::mt19937 gen;
    std::uniform_real_distribution<float> uniform_dist;
    std::normal_distribution<float> normal_dist;
    
public:
    RandomGenerator(unsigned int seed = 123456) : gen(seed), uniform_dist(0.0f, 1.0f), normal_dist(0.0f, 1.0f) {}
    
    float uniform(float min = 0.0f, float max = 1.0f) {
        return min + (max - min) * uniform_dist(gen);
    }
    
    float normal(float mean = 0.0f, float stddev = 1.0f) {
        return mean + stddev * normal_dist(gen);
    }
};

} // namespace dogm
