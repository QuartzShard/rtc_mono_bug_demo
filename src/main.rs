//! Minimum viable demo of atsamd-hal RTC monotonic time loss bug.
//!
//! # Bug Summary
//!
//! The `RtcBackend` implementation of `TimerQueueBackend::disable_timer()` in
//! `atsamd-hal` clears `RTC.MODE0.CTRLA.ENABLE`, which **stops the hardware
//! counter**. The `rtic-time` `TimerQueue` calls `disable_timer()` whenever the
//! queue is empty (as a power-saving optimisation). This means any period where
//! no `RtcMono::delay()` is pending causes the RTC counter to freeze, and all
//! wall-clock time elapsed while stopped is permanently lost.
//!
//! The `rtic-time` `TimerQueueBackend` trait documents `disable_timer()` as:
//!
//! > "Enabling and disabling the monotonic needs to propagate to `now` so that
//! > an instant based of `now()` is still valid."
//!
//! The current implementation violates this contract — it freezes the counter
//! without compensating `now()`.
//!
//! # Running
//!
//! ```sh
//! # Show the bug: timer queue empties during busy-wait, RTC counter freezes
//! cargo embed
//!
//! # Show the workaround: a long-lived delay keeps the queue non-empty
//! cargo embed --features workaround
//! ```
//!
//! # Expected Output
//!
//! Without `workaround`, the RTC reports ~0ms for the busy phase while SysTick
//! delays for a known duration. With the `workaround` feature, the RTC matches
//! the SysTick reference.
//!
//! # Target
//!
//! ATSAMD51J20A (Cortex-M4F), XOSC32K crystal on PA00/PA01, plain-text RTT.

#![no_std]
#![no_main]

use panic_halt as _;

/// CPU frequency in Hz. Default DFLL at reset, no DPLL configured.
const CPU_HZ: u32 = 48_000_000;

/// Duration of the busy-wait phase in milliseconds.
const BUSY_WAIT_MS: u32 = 5_000;

#[rtic::app(device = atsamd_hal::pac, dispatchers = [TC0, TC1])]
mod app {
    use super::*;
    use atsamd_hal::{
        clock::v2::{
            clock_system_at_reset,
            rtcosc::RtcOsc,
            xosc32k::{ControlGainMode, StartUpDelay, Xosc32k, Xosc32kBase},
        },
        fugit::ExtU64 as _,
        gpio,
        rtc::rtic::rtc_clock,
        rtc_monotonic,
        rtic_time::Monotonic,
    };
    use cortex_m::delay::Delay;
    use rtt_target::{rprintln, rtt_init_print};

    rtc_monotonic!(RtcMono, rtc_clock::Clock32k);

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        syst_delay: Delay,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        rtt_init_print!();

        let syst_delay = Delay::new(ctx.core.SYST, CPU_HZ);

        let mut device = ctx.device;

        // Minimum clock setup: XOSC32K crystal -> RTC monotonic
        let (_bus, _clocks, tokens) = clock_system_at_reset(
            device.oscctrl,
            device.osc32kctrl,
            device.gclk,
            device.mclk,
            &mut device.nvmctrl,
        );

        let pins = gpio::Pins::new(device.port);
        let xosc32k_base = Xosc32kBase::from_crystal(tokens.xosc32k.base, pins.pa00, pins.pa01)
            .start_up_delay(StartUpDelay::Delay1s)
            .control_gain_mode(ControlGainMode::HighSpeed)
            .on_demand(false)
            .run_standby(true)
            .enable();
        while !xosc32k_base.is_ready() {
            cortex_m::asm::nop();
        }
        let (xosc32k, xosc32k_base) = Xosc32k::enable(tokens.xosc32k.xosc32k, xosc32k_base);
        xosc32k_base.write_lock();

        let (_rtc_osc, _xosc32k) = RtcOsc::enable(tokens.rtcosc, xosc32k);
        RtcMono::start(device.rtc);

        rprintln!("=== atsamd-hal RTC disable_timer() bug demo ===");
        rprintln!("RTC monotonic: 32768 Hz (XOSC32K)");
        rprintln!();
        rprintln!(
            "Each iteration: 1s async delay, then {}ms SysTick busy-wait.",
            BUSY_WAIT_MS
        );
        rprintln!(
            "RTC should report ~{}ms total. Compare with SysTick reference.",
            1000 + BUSY_WAIT_MS
        );
        rprintln!();

        #[cfg(not(feature = "workaround"))]
        {
            rprintln!("MODE: BUG — timer queue empties during busy-wait,");
            rprintln!("      disable_timer() stops the RTC counter.");
        }
        #[cfg(feature = "workaround")]
        {
            rprintln!("MODE: WORKAROUND — perpetual delay keeps timer queue non-empty.");
            keep_queue_alive::spawn().ok();
        }

        rprintln!();
        demo_task::spawn().ok();

        (Shared {}, Local { syst_delay })
    }

    /// Workaround: a single very long delay that keeps the timer queue
    /// permanently non-empty, preventing `disable_timer()` from ever firing.
    #[task(priority = 1)]
    async fn keep_queue_alive(_ctx: keep_queue_alive::Context) {
        RtcMono::delay(24u64.hours()).await;
    }

    /// Alternates between an async RtcMono delay and a CPU busy-wait.
    ///
    /// During the async delay, the timer queue is non-empty and the RTC runs.
    /// During the busy-wait, no delays are pending. The timer queue empties,
    /// `disable_timer()` stops the RTC, and wall-clock time is lost.
    #[task(local = [syst_delay], priority = 1)]
    async fn demo_task(ctx: demo_task::Context) {
        // Allow RTT connection to settle
        RtcMono::delay(1u64.secs()).await;

        let syst = ctx.local.syst_delay;
        let mut iteration = 0u32;
        loop {
            iteration += 1;

            let t0 = RtcMono::now();

            // 1-second async delay. Timer queue has an entry -> RTC runs.
            RtcMono::delay(1u64.secs()).await;

            let t1 = RtcMono::now();
            let delay_rtc = t1 - t0;

            // SysTick busy-wait for a known duration. No pending RtcMono delays
            // -> timer queue empties -> disable_timer() fires -> RTC freezes.
            syst.delay_ms(BUSY_WAIT_MS);

            let t2 = RtcMono::now();
            let busy_rtc = t2 - t1;
            let total_rtc = t2 - t0;

            rprintln!("[iter {}]          RTC    SysTick", iteration);
            rprintln!(
                "  delay phase: {:>7}ms {:>7}ms",
                delay_rtc.to_millis(),
                1000
            );
            rprintln!(
                "  busy phase:  {:>7}ms {:>7}ms",
                busy_rtc.to_millis(),
                BUSY_WAIT_MS
            );
            rprintln!(
                "  total:       {:>7}ms {:>7}ms",
                total_rtc.to_millis(),
                1000 + BUSY_WAIT_MS
            );
            rprintln!();
        }
    }
}
