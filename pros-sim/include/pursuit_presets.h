#pragma once

#include "pure_pursuit.h"

// --------------------------------------------------------------------------
//  pursuit_presets.h - ready-to-use FollowPathParams presets
//
//  Usage (in autonomous):
//
//    #include "pursuit_presets.h"
//
//    // Pick a preset and go:
//    actor.followPath(path, pursuit, PursuitPreset::FAST);
//    actor.followPath(path, pursuit, PursuitPreset::PRECISE);
//
//    // Chain two paths together smoothly:
//    actor.followPath(path1, pursuit, PursuitPreset::CHAIN_FAST);
//    actor.followPath(path2, pursuit, PursuitPreset::PRECISE);
//
//    // Or tweak a preset inline:
//    auto p = PursuitPreset::BALANCED;
//    p.maxSpeed = 50.0;
//    actor.followPath(path, pursuit, p);
//
// --------------------------------------------------------------------------

namespace PursuitPreset {

  // ========================================================================
  //  Standalone presets (robot stops at end)
  // ========================================================================

  /// Default balanced profile - good starting point for most paths.
  inline constexpr FollowPathParams BALANCED = {
    .forwards = true,
    .maxSpeed = 30.0,
    .minSpeed = 0.0,
    .earlyExitRange = 0.0,
    .slewRate = 800.0,
    .lookahead = 5.0,
  };

  /// Full speed - for long straight-heavy paths.
  inline constexpr FollowPathParams FAST = {
    .forwards = true,
    .maxSpeed = 100.0,
    .minSpeed = 0.0,
    .earlyExitRange = 0.0,
    .slewRate = 1200.0,
    .lookahead = 20.0,
  };

  /// Precise / slow - for tight turns or accuracy.
  inline constexpr FollowPathParams PRECISE = {
    .forwards = true,
    .maxSpeed = 45.0,
    .minSpeed = 0.0,
    .earlyExitRange = 0.0,
    .slewRate = 600.0,
    .lookahead = 10.0,
  };

  /// Reverse balanced.
  inline constexpr FollowPathParams REVERSE = {
    .forwards = false,
    .maxSpeed = 70.0,
    .minSpeed = 0.0,
    .earlyExitRange = 0.0,
    .slewRate = 800.0,
    .lookahead = 15.0,
  };

  /// Reverse fast.
  inline constexpr FollowPathParams REVERSE_FAST = {
    .forwards = false,
    .maxSpeed = 100.0,
    .minSpeed = 0.0,
    .earlyExitRange = 0.0,
    .slewRate = 1200.0,
    .lookahead = 20.0,
  };

  /// Reverse precise.
  inline constexpr FollowPathParams REVERSE_PRECISE = {
    .forwards = false,
    .maxSpeed = 45.0,
    .minSpeed = 0.0,
    .earlyExitRange = 0.0,
    .slewRate = 600.0,
    .lookahead = 10.0,
  };

  // ========================================================================
  //  Motion-chaining presets (robot keeps rolling into next motion)
  // ========================================================================

  /// Chain at moderate speed.
  inline constexpr FollowPathParams CHAIN = {
    .forwards = true,
    .maxSpeed = 70.0,
    .minSpeed = 20.0,
    .earlyExitRange = 3.0,
    .slewRate = 800.0,
    .lookahead = 15.0,
  };

  /// Chain at full speed.
  inline constexpr FollowPathParams CHAIN_FAST = {
    .forwards = true,
    .maxSpeed = 100.0,
    .minSpeed = 30.0,
    .earlyExitRange = 4.0,
    .slewRate = 1200.0,
    .lookahead = 20.0,
  };

  /// Chain slowly.
  inline constexpr FollowPathParams CHAIN_PRECISE = {
    .forwards = true,
    .maxSpeed = 45.0,
    .minSpeed = 15.0,
    .earlyExitRange = 2.0,
    .slewRate = 600.0,
    .lookahead = 10.0,
  };

  /// Reverse chain.
  inline constexpr FollowPathParams CHAIN_REVERSE = {
    .forwards = false,
    .maxSpeed = 70.0,
    .minSpeed = 20.0,
    .earlyExitRange = 3.0,
    .slewRate = 800.0,
    .lookahead = 15.0,
  };

} // namespace PursuitPreset
