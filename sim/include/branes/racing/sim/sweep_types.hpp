#pragma once
/// @file sweep_types.hpp
/// @brief The scalar types of the precision sweep (docs/plan/precision-sweep/).
///
/// One list for precision_sweep_<baseline> and the tests of the casts and
/// tolerances, so both cover the same types. Including it needs Universal
/// (universal::universal); the rest of sim/ does not.

#include <universal/number/cfloat/cfloat.hpp>
#include <universal/number/posit/posit.hpp>

#include <cstdint>
#include <string_view>
#include <tuple>

namespace branes::racing::sim {

// IEEE layouts in software: subnormals, no max-exponent values, no saturation.
using cfloat32 = sw::universal::cfloat<32, 8, std::uint32_t, true, false, false>;
using bfloat16 = sw::universal::cfloat<16, 8, std::uint16_t, true, false, false>;
using binary16 = sw::universal::cfloat<16, 5, std::uint16_t, true, false, false>;
using posit32 = sw::universal::posit<32, 2>;
using posit16 = sw::universal::posit<16, 1>;

/// The sweep's types, in order; double first, the reference.
using SweepTypes = std::tuple<double, float, cfloat32, posit32, bfloat16, binary16, posit16>;

/// A sweep type's file name (slug) and its label in the results table.
template <typename T>
struct SweepType {
    std::string_view slug;
    std::string_view label;
};

/// The names of SweepTypes, element for element.
inline const auto kSweepTypes = std::tuple{SweepType<double>{"double", "double"},
                                           SweepType<float>{"float", "float"},
                                           SweepType<cfloat32>{"cfloat32_8", "cfloat<32,8>"},
                                           SweepType<posit32>{"posit32_2", "posit<32,2>"},
                                           SweepType<bfloat16>{"bfloat16", "bfloat16 (cfloat<16,8>)"},
                                           SweepType<binary16>{"binary16", "binary16 (cfloat<16,5>)"},
                                           SweepType<posit16>{"posit16_1", "posit<16,1>"}};

}  // namespace branes::racing::sim
