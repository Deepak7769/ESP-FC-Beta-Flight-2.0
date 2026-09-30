#pragma once

// -----------------------------------------------------------------------------
// Assisted-mode V2 production feature selection.
//
// ANGLE V2 is the only Angle controller and is always compiled.
// ESPFC_ASSISTED_V2_ACTIVE promotes AltHold V2 and LAND V2 to the
// authoritative vertical/failsafe controllers. The standard ESP32 production
// environment defines this macro.
//
// A motor-driving AltHold/LAND build must explicitly acknowledge actuator
// authority through ESPFC_ASSISTED_V2_OUTPUT_ACK.
// -----------------------------------------------------------------------------



#if defined(ESPFC_ALTHOLD_V2_ACTIVE_TEST) && \
    !defined(ESPFC_SAFE_BENCH_BUILD)
#error "ESPFC_ALTHOLD_V2_ACTIVE_TEST requires ESPFC_SAFE_BENCH_BUILD"
#endif

#if defined(ESPFC_LAND_V2_ACTIVE_TEST) && \
    !defined(ESPFC_SAFE_BENCH_BUILD)
#error "ESPFC_LAND_V2_ACTIVE_TEST requires ESPFC_SAFE_BENCH_BUILD"
#endif

#if defined(ESPFC_ASSISTED_V2_ACTIVE) && \
    !defined(ESPFC_SAFE_BENCH_BUILD) && \
    !defined(ESPFC_ASSISTED_V2_OUTPUT_ACK)
#error "Motor-driving Assisted V2 builds require ESPFC_ASSISTED_V2_OUTPUT_ACK"
#endif

#if defined(ESPFC_ASSISTED_V2_ACTIVE) || \
    defined(ESPFC_ALTHOLD_V2_ACTIVE_TEST)
#define ESPFC_ALTHOLD_V2_ACTIVE 1
#endif

#if defined(ESPFC_ASSISTED_V2_ACTIVE) || \
    defined(ESPFC_LAND_V2_ACTIVE_TEST)
#define ESPFC_LAND_V2_ACTIVE 1
#endif

#if defined(ESPFC_LAND_V2_ACTIVE) && \
    !defined(ESPFC_ALTHOLD_V2_ACTIVE)
#error "LAND V2 requires AltHold V2"
#endif
// Channel index carrying a spring-centered vertical-stick command for AltHold.
// 3 == the normal throttle channel and preserves existing bench/test behavior.
// The production controller must explicitly provide a separate centered
// vertical command; the current project uses AUX3 / input index 6.
#if defined(ESPFC_ASSISTED_V2_ACTIVE) && \
    !defined(ESPFC_SAFE_BENCH_BUILD) && \
    !defined(ESPFC_ALTHOLD_V2_CENTERED_STICK_CHANNEL)
#error "Production Assisted V2 requires an explicit centered AltHold stick channel"
#endif

#ifndef ESPFC_ALTHOLD_V2_CENTERED_STICK_CHANNEL
#define ESPFC_ALTHOLD_V2_CENTERED_STICK_CHANNEL 3
#endif

#if defined(ESPFC_ASSISTED_V2_ACTIVE) && \
    !defined(ESPFC_SAFE_BENCH_BUILD) && \
    ESPFC_ALTHOLD_V2_CENTERED_STICK_CHANNEL == 3
#error "Production Assisted V2 cannot use the stateful throttle channel as the centered AltHold stick"
#endif

#if ESPFC_ALTHOLD_V2_CENTERED_STICK_CHANNEL < 0 || \
    ESPFC_ALTHOLD_V2_CENTERED_STICK_CHANNEL > 15
#error "ESPFC_ALTHOLD_V2_CENTERED_STICK_CHANNEL must be in range 0..15"
#endif

// Positive LAND V2 descent speed in m/s. The controller applies the negative
// sign for downward motion. The project production default is deliberately
// conservative at 0.10 m/s unless a build policy overrides it.
#ifndef ESPFC_LAND_V2_DESCENT_RATE_MS
#define ESPFC_LAND_V2_DESCENT_RATE_MS 0.10f
#endif
