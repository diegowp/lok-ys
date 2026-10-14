#pragma once

#include <cstdint>
#include <cstddef>
#include <concepts>
#include <span>

namespace LokYS {

// Fixed-width integer aliases
using u8  = std::uint8_t;
using u16 = std::uint16_t;
using u32 = std::uint32_t;
using u64 = std::uint64_t;

using i8  = std::int8_t;
using i16 = std::int16_t;
using i32 = std::int32_t;
using i64 = std::int64_t;

using f32 = float;
using f64 = double;

// Fundamental simulation timing constants
inline constexpr f64 FIXED_TIMESTEP_SEC = 1.0 / 120.0; // 120 Hz fixed physics rate
inline constexpr u32 MAX_SUB_STEPS_PER_FRAME = 5;       // Clamped accumulator limit

// Strong-typing concept constraints
template <typename T>
concept Numeric = std::integral<T> || std::floating_point<T>;

} // namespace LokYS
