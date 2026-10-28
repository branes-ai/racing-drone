#pragma once
/// @file sitl_loop.hpp
/// @brief run_sitl(): the SITL closed loop that writes the log, for sitl_<baseline>.
///
/// Mission-neutral (docs/assessments/framework-boundary.md). Every simulator
/// event is logged, fed to the stack through the Runner, and every output is
/// logged; each command goes back to the simulated FC. The mission supplies
/// the simulator, the stack, and its pass-through channels (racing:
/// apps/sitl/main.cpp).

#include <branes/racing/compose/tick.hpp>
#include <branes/racing/harness/runner.hpp>
#include <branes/racing/log/log.hpp>
#include <branes/racing/msg/codec.hpp>
#include <branes/racing/result.hpp>
#include <branes/racing/sim/basic_simulator.hpp>

#include <cstdint>
#include <span>
#include <string>
#include <utility>

namespace branes::racing::sim {

/// Run `sim` and `stack` to the end, logging every input and output to
/// `writer`. Returns the number of ticks, or the error that stopped the run.
/// Pre: writer.is_open().
template <class Oracle, RunnableStack StackT>
[[nodiscard]] result<std::uint64_t, std::string>
run_sitl(BasicSimulator<Oracle>& sim, StackT& stack, std::span<const log::Channel> mission, log::LogWriter& writer) {
    harness::Runner runner(stack, mission);
    std::uint64_t ticks = 0;
    while (auto ev = sim.next()) {
        writer.write(ev->channel, ev->t, ev->payload);
        auto outputs = runner.feed(log::LogRecord{ev->channel, ev->t, std::move(ev->payload)});
        if (!outputs) {
            return outputs.error();
        }
        for (const auto& o : *outputs) {
            writer.write(o.channel, o.t, o.payload);
            if (o.channel == log::Channel::command) {
                // The command the stack sent, back to the simulated FC.
                // decode(encode(u)) == u: the codec writes every double exactly.
                const auto u = codec::decode_command(o.payload);
                if (!u) {
                    return std::string("the stack produced a command that does not decode");
                }
                sim.command(*u);
            }
        }
        if (ev->channel == log::Channel::tick) {
            ++ticks;
        }
    }
    return ticks;
}

}  // namespace branes::racing::sim
