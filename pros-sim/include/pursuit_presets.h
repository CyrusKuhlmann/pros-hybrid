#pragma once

#include "pure_pursuit.h"

// ─────────────────────────────────────────────────────────────────────
//  pursuit_presets.h – ready-to-use FollowPathParams presets
//
//  Usage (in autonomous):
//
//    #include "pursuit_presets.h"
//
//    // Just pick a preset and go:
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
// ─────────────────────────────────────────────────────────────────────

namespace PursuitPreset {

  // ═════════════════════════════════════════════════════════════════════
  //  Standalone presets (robot stops at end)
  // ═════════════════════════════════════════════════════════════════════

  /// Default balanced profile – good starting point for most paths.
  ///   Moderate speed, smooth deceleration, gentle curve limiting.
  inline constexpr FollowPathParams BALANCED = {
    .forwards = true,
    .maxSpeed = 70.0,
    .minSpeed = 0.0,
    .earlyExitRange = 0.0,
    .maxAccel = 4.0,
    .decelDist = 12.0,
    .maxCentripetalAccel = 50.0,
  };

  /// Full speed – for long straight-heavy paths where time matters.
  ///   High speed, longer decel ramp, generous centripetal budget.
  inline constexpr FollowPathParams FAST = {
    .forwards = true,
    .maxSpeed = 100.0,
    .minSpeed = 0.0,
    .earlyExitRange = 0.0,
    .maxAccel = 6.0,
    .decelDist = 18.0,
    .maxCentripetalAccel = 80.0,
  };

  /// Precise / slow – for tight turns or paths that need accuracy.
  ///   Low speed, aggressive curve limiting, short decel.
  inline constexpr FollowPathParams PRECISE = {
    .forwards = true,
    .maxSpeed = 45.0,
    .minSpeed = 0.0,
    .earlyExitRange = 0.0,
    .maxAccel = 3.0,
    .decelDist = 8.0,
    .maxCentripetalAccel = 30.0,
  };

  /// Reverse balanced – same as BALANCED but driving backwards.
  inline constexpr FollowPathParams REVERSE = {
    .forwards = false,
    .maxSpeed = 70.0,
    .minSpeed = 0.0,
    .earlyExitRange = 0.0,
    .maxAccel = 4.0,
    .decelDist = 12.0,
    .maxCentripetalAccel = 50.0,
  };

  /// Reverse fast – full speed backwards.
  inline constexpr FollowPathParams REVERSE_FAST = {
    .forwards = false,
    .maxSpeed = 100.0,
    .minSpeed = 0.0,
    .earlyExitRange = 0.0,
    .maxAccel = 6.0,
    .decelDist = 18.0,
    .maxCentripetalAccel = 80.0,
  };

  // ═════════════════════════════════════════════════════════════════════
  //  Motion-chaining presets (robot keeps rolling into next motion)
  //
  //  Use these for the FIRST path in a chain, then finish with a
  //  standalone preset (BALANCED, PRECISE, etc.) for the last path.
  // ═════════════════════════════════════════════════════════════════════

  /// Chain at moderate speed – smooth handoff to next motion.
  ///   20% floor speed, exits 3" early.
  inline constexpr FollowPathParams CHAIN = {
    .forwards = true,
    .maxSpeed = 70.0,
    .minSpeed = 20.0,
    .earlyExitRange = 3.0,
    .maxAccel = 4.0,
    .decelDist = 12.0,
    .maxCentripetalAccel = 50.0,
  };

  /// Chain at full speed – for fast autonomous routines.
  ///   30% floor speed, exits 4" early.
  inline constexpr FollowPathParams CHAIN_FAST = {
    .forwards = true,
    .maxSpeed = 100.0,
    .minSpeed = 30.0,
    .earlyExitRange = 4.0,
    .maxAccel = 6.0,
    .decelDist = 12.0,
    .maxCentripetalAccel = 80.0,
  };

  /// Chain slowly – for careful chaining around obstacles.
  ///   15% floor speed, exits 2" early.
  inline constexpr FollowPathParams CHAIN_PRECISE = {
    .forwards = true,
    .maxSpeed = 45.0,
    .minSpeed = 15.0,
    .earlyExitRange = 2.0,
    .maxAccel = 3.0,
    .decelDist = 8.0,
    .maxCentripetalAccel = 30.0,
  };

  /// Reverse chain – chaining while driving backwards.
  inline constexpr FollowPathParams CHAIN_REVERSE = {
    .forwards = false,
    .maxSpeed = 70.0,
    .minSpeed = 20.0,
    .earlyExitRange = 3.0,
    .maxAccel = 4.0,
    .decelDist = 12.0,
    .maxCentripetalAccel = 50.0,
  };

  // ═════════════════════════════════════════════════════════════════════
  //  Controller tuning presets
  //
  //  Apply to a PurePursuitController to set adaptive / cross-track
  //  behavior.  Call like:
  //
  //    PurePursuitController pursuit(15, 11.67);
  //    PursuitPreset::applyTight(pursuit);
  // ═════════════════════════════════════════════════════════════════════

  /// Balanced controller tuning – good default.
  ///   Moderate adaptive K, no cross-track correction.
  inline void applyBalanced(PurePursuitController& ctrl) {
    ctrl.setLookahead(15.0);
    ctrl.setAdaptiveK(5.0);
    ctrl.setMinLookahead(6.0);
    ctrl.setCrossTrackGain(0.0);
  }

  /// Tight controller tuning – for paths with sharp curves.
  ///   Aggressive adaptive shrink, shorter lookahead,
  ///   light cross-track correction.
  inline void applyTight(PurePursuitController& ctrl) {
    ctrl.setLookahead(10.0);
    ctrl.setAdaptiveK(8.0);
    ctrl.setMinLookahead(4.0);
    ctrl.setCrossTrackGain(0.02);
  }

  /// Smooth controller tuning – for long sweeping paths.
  ///   Large lookahead, gentle adaptive, no cross-track.
  inline void applySmooth(PurePursuitController& ctrl) {
    ctrl.setLookahead(20.0);
    ctrl.setAdaptiveK(3.0);
    ctrl.setMinLookahead(8.0);
    ctrl.setCrossTrackGain(0.0);
  }

} // namespace PursuitPreset
