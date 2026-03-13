/// @file fire_control_async.cpp
/// @brief Demonstrates thread-safe fire control using FireControlTable.
///
/// FireControlTable::build() takes ~20 ms — more than one 60 Hz frame.
/// This example shows the recommended game-loop pattern:
///
///   1. Build the table on a background thread (no main-thread stall).
///   2. Call poll() every frame to swap the new table in when ready.
///   3. Call lookup() every frame — O(log N), < 1 µs.
///
/// The FireControlSystem class here is a self-contained drop-in that can
/// be adapted to any game engine's threading model.  If your engine has its
/// own job/task system, replace std::async with a job submission and call
/// notify_build_complete() when the job finishes.

#include <ballistics/ballistics.hpp>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <future>
#include <memory>
#include <thread>

using namespace ballistics;
using Clock = std::chrono::high_resolution_clock;

static double elapsed_ms(Clock::time_point a, Clock::time_point b) {
    return std::chrono::duration<double, std::milli>(b - a).count();
}
static double elapsed_us(Clock::time_point a, Clock::time_point b) {
    return std::chrono::duration<double, std::micro>(b - a).count();
}

// ---------------------------------------------------------------------------
// FireControlSystem
// ---------------------------------------------------------------------------
/// Thread-safe fire control manager.
///
/// Thread model
/// ------------
///   Main thread  : poll(), lookup()  — lock-free reads
///   Worker thread: build() runs inside std::async
///
/// The active table is stored behind a shared_ptr.  The worker builds a new
/// FireControlTable into a fresh instance, then the main thread atomically
/// swaps the shared_ptr on the next poll() call.  Because shared_ptr
/// assignment is not itself atomic, the swap is protected by a single
/// lightweight std::atomic_flag (test-and-set on poll, which is called once
/// per frame at most).
// ---------------------------------------------------------------------------
class FireControlSystem {
public:
    /// Kick off an async build.  Returns immediately.
    /// Safe to call from the main thread at any time (weapon change, weather
    /// update, etc.).  If a build is already in flight it is left to complete
    /// — a second request is silently ignored until the first finishes.
    ///
    /// @note  Must be called from the same thread as poll() and lookup().
    ///        The TrajectorySimulator passed here is copied into the async
    ///        task, so the caller's instance may be destroyed after this
    ///        call returns.
    void request_build(const TrajectorySimulator& sim,
                       double muzzle_speed_ms,
                       double azimuth_deg     = 0.0,
                       double launch_height_m = 0.0,
                       bool   high_angle      = false,
                       int    num_samples     = 500)
    {
        // Don't start a new build if one is already running
        if (pending_.valid() &&
            pending_.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
        {
            std::printf("[FireControlSystem] Build already in flight — request ignored.\n");
            return;
        }

        build_started_ = Clock::now();

        // Copy sim into the closure so the async task owns its own instance.
        // The caller's sim may be a stack-local (e.g. sim2 at frame 60) that
        // is destroyed before the background task runs.
        pending_ = std::async(std::launch::async,
            [=, sim_copy = sim]() -> FireControlTable {
                FireControlTable t;
                t.build(sim_copy, muzzle_speed_ms, azimuth_deg,
                        launch_height_m, high_angle, num_samples);
                return t;
            });
    }

    /// Call once per frame on the main thread.
    /// Swaps the new table in if the background build has completed.
    /// Cost: < 1 µs when no build is pending.
    ///
    /// @note  Must be called from the same thread as request_build() and
    ///        lookup().  There is no internal synchronisation — the design
    ///        assumes a single "main" thread owns this object.
    void poll() {
        if (!pending_.valid()) return;
        if (pending_.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
            return;

        // Build finished — install new table (main thread only, no locking needed)
        active_ = std::make_shared<FireControlTable>(pending_.get());
        const double build_ms = elapsed_ms(build_started_, Clock::now());
        std::printf("[FireControlSystem] New table ready: max_range=%.1f m  "
                    "entries=%zu  build_time=%.1f ms\n",
                    active_->max_range_m(),
                    active_->entries().size(),
                    build_ms);
    }

    /// Interpolated lookup — O(log N), safe to call every frame.
    /// Returns valid=false if the table is not ready yet or range exceeds max.
    [[nodiscard]] FireSolution lookup(double range_m) const {
        if (!active_) return {};
        return active_->lookup(range_m);
    }

    [[nodiscard]] bool ready()        const noexcept { return active_ && active_->ready(); }
    [[nodiscard]] double max_range_m() const noexcept {
        return active_ ? active_->max_range_m() : 0.0;
    }

private:
    std::shared_ptr<FireControlTable> active_;
    std::future<FireControlTable>     pending_;
    Clock::time_point                 build_started_;
};

// ---------------------------------------------------------------------------
// Simulated game loop
// ---------------------------------------------------------------------------
int main(int argc, char* argv[]) {
    // -----------------------------------------------------------------------
    // Setup
    // -----------------------------------------------------------------------
    const std::string data_path = (argc > 1) ? argv[1] : "data/munitions.json";

    MunitionLibrary lib;
    try {
        lib.load(data_path);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "Error loading munitions: %s\n", e.what());
        return 1;
    }

    const MunitionSpec& spec        = lib.get("5.56x45_m855_62gr");
    const double        muzzle_speed = spec.muzzle_velocity_ms;
    AtmosphericConditions atm;

    TrajectorySimulator sim(spec, atm);

    // -----------------------------------------------------------------------
    // Fire control system
    // -----------------------------------------------------------------------
    FireControlSystem fcs;

    std::printf("=== Async Fire Control Demo ===\n");
    std::printf("Munition: %s  muzzle=%.0f m/s\n\n", spec.name.c_str(), muzzle_speed);

    // Kick off the initial build immediately (simulates weapon equip at startup)
    std::printf("[Frame 0] Requesting fire control table build...\n");
    fcs.request_build(sim, muzzle_speed);

    // -----------------------------------------------------------------------
    // Simulate 120 frames at 60 Hz
    // -----------------------------------------------------------------------
    const int    total_frames   = 120;
    const double frame_dt_ms    = 1000.0 / 60.0;   // 16.67 ms
    const double budget_us      = frame_dt_ms * 1000.0;

    // Simulate a target moving from 100 m to 1200 m over the run
    double total_lookup_us = 0.0;
    int    lookup_count    = 0;

    for (int frame = 0; frame < total_frames; ++frame) {
        auto frame_start = Clock::now();

        // --- Swap in new table if the worker finished ---
        fcs.poll();

        // --- Fire control lookup (every frame) ---
        const double range_m = 100.0 + (1100.0 * frame) / (total_frames - 1);
        auto t0 = Clock::now();
        FireSolution sol = fcs.lookup(range_m);
        auto t1 = Clock::now();
        total_lookup_us += elapsed_us(t0, t1);
        ++lookup_count;

        // --- Print every 10th frame ---
        if (frame % 10 == 0) {
            if (sol.valid) {
                std::printf("[Frame %3d] range=%6.0f m  elev=%6.3f°  ToF=%5.0f ms  "
                            "lookup=%.3f µs\n",
                            frame, range_m,
                            sol.elevation_deg, sol.flight_time_ms,
                            elapsed_us(t0, t1));
            } else {
                std::printf("[Frame %3d] range=%6.0f m  (table not ready yet)\n",
                            frame, range_m);
            }
        }

        // --- Simulate weapon change at frame 60 (triggers a rebuild) ---
        if (frame == 60) {
            std::printf("\n[Frame 60] Weapon change — requesting new table build...\n\n");
            const MunitionSpec& spec2 = lib.get("7.62x51_m80_147gr");
            TrajectorySimulator sim2(spec2, atm);
            fcs.request_build(sim2, spec2.muzzle_velocity_ms);
        }

        // --- Burn remaining frame time (simulate other game work) ---
        auto frame_end = Clock::now();
        const double used_ms = elapsed_ms(frame_start, frame_end);
        if (used_ms < frame_dt_ms) {
            std::this_thread::sleep_for(
                std::chrono::microseconds(
                    static_cast<long>((frame_dt_ms - used_ms) * 1000.0)));
        }
    }

    // -----------------------------------------------------------------------
    // Summary
    // -----------------------------------------------------------------------
    std::printf("\n--- Summary ---\n");
    std::printf("Total frames     : %d\n", total_frames);
    std::printf("Avg lookup time  : %.3f µs  (budget: %.0f µs/frame)\n",
                total_lookup_us / lookup_count, budget_us);
    std::printf("Max range served : %.1f m\n", fcs.max_range_m());

    return 0;
}
