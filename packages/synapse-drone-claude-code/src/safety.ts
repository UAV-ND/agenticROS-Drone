/**
 * Drone action safety guard.
 *
 * The agent never publishes raw velocities — it calls coarse drone verbs that
 * map to ROS 2 actions. This module validates those action inputs before they
 * reach the transport so the safety supervisor (N11) gets clean, bounded calls.
 *
 * Rules enforced here (soft bounds; the ArduPilot safety supervisor is the
 * hard backstop):
 *   - Takeoff altitude must be positive and within the configured ceiling.
 *   - Dice color must be one of the four competition colors.
 *   - Navigation targets must be finite numbers (NaN/Inf blocked).
 */

const COMPETITION_DICE_COLORS = new Set(["red", "blue", "yellow", "black"]);

// TODO: ICD N3 — confirm altitude ceiling with GNC once signed (target: Nov 6)
const DEFAULT_MAX_ALTITUDE_M = 10.0;
const DEFAULT_MIN_ALTITUDE_M = 0.3;
const DEFAULT_TAKEOFF_ALTITUDE_M = 3.0;

export interface SafetyCheckResult {
  ok: boolean;
  reason?: string;
}

export function checkTakeoffAltitude(
  altitude_m: number,
  maxAltitude = DEFAULT_MAX_ALTITUDE_M,
): SafetyCheckResult {
  if (!Number.isFinite(altitude_m) || altitude_m <= 0) {
    return { ok: false, reason: `altitude_m must be a positive finite number, got ${altitude_m}` };
  }
  if (altitude_m < DEFAULT_MIN_ALTITUDE_M) {
    return { ok: false, reason: `altitude_m ${altitude_m} m is below minimum ${DEFAULT_MIN_ALTITUDE_M} m` };
  }
  if (altitude_m > maxAltitude) {
    return { ok: false, reason: `altitude_m ${altitude_m} m exceeds ceiling ${maxAltitude} m` };
  }
  return { ok: true };
}

export function checkDiceColor(color: string): SafetyCheckResult {
  const normalized = color.trim().toLowerCase();
  if (!COMPETITION_DICE_COLORS.has(normalized)) {
    return {
      ok: false,
      reason: `"${color}" is not a valid competition dice color. Must be one of: ${[...COMPETITION_DICE_COLORS].join(", ")}.`,
    };
  }
  return { ok: true };
}

export function checkNavTarget(x: number, y: number, z?: number): SafetyCheckResult {
  if (!Number.isFinite(x) || !Number.isFinite(y)) {
    return { ok: false, reason: `Navigation target x=${x}, y=${y} must be finite numbers` };
  }
  if (z !== undefined && !Number.isFinite(z)) {
    return { ok: false, reason: `Navigation altitude z=${z} must be a finite number` };
  }
  return { ok: true };
}

export { DEFAULT_TAKEOFF_ALTITUDE_M };
