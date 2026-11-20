/**
 * Mission 3 flow for mid-mission target switching.
 *
 * The project keeps the expanded drone API. This file provides a concrete,
 * stateful pattern for handling a dice-color change in the middle of a run.
 *
 * The flow is:
 *   1. search using the current target color
 *   2. detect a higher-priority or newly visible color
 *   3. call Change_route(new_dice_color)
 *   4. cancel the old search/approach
 *   5. re-plan from the current pose toward the new target
 *   6. continue until the mission is complete or the drone returns home
 */

import { checkDiceColor } from "./safety.js";

export type DiceColor = "red" | "blue" | "yellow" | "black";

export type Mission3State = {
  activeColor: DiceColor | null;
  phase: "idle" | "searching" | "approaching" | "returning";
  lastKnownPose?: { x: number; y: number };
  lastUpdatedAt?: number;
};

export const DEFAULT_MISSION3_STATE: Mission3State = {
  activeColor: "red",
  phase: "searching",
};

export function createMission3State(initialColor: DiceColor = "red"): Mission3State {
  return {
    ...DEFAULT_MISSION3_STATE,
    activeColor: initialColor,
  };
}

export function updateMission3Target(
  state: Mission3State,
  newColor: DiceColor,
  pose?: { x: number; y: number },
): Mission3State {
  return {
    ...state,
    activeColor: newColor,
    phase: "searching",
    lastKnownPose: pose ?? state.lastKnownPose,
    lastUpdatedAt: Date.now(),
  };
}

export function applyRouteChange(
  state: Mission3State,
  newDiceColor: string,
  pose?: { x: number; y: number },
): { ok: true; state: Mission3State } | { ok: false; reason: string } {
  const route = validateRouteChange(newDiceColor);
  if (!route.ok) {
    return route;
  }

  return {
    ok: true,
    state: updateMission3Target(state, route.color, pose),
  };
}

export function normalizeDiceColor(value: string): DiceColor | null {
  const normalized = value.trim().toLowerCase();
  if (normalized === "red" || normalized === "blue" || normalized === "yellow" || normalized === "black") {
    return normalized;
  }
  return null;
}

export function shouldReTarget(currentColor: DiceColor | null, detectedColor: string | null): boolean {
  if (!currentColor || !detectedColor) return false;
  const normalized = normalizeDiceColor(detectedColor);
  if (!normalized) return false;
  return normalized !== currentColor;
}

export function buildRouteChangeSummary(oldColor: DiceColor | null, newColor: DiceColor): string {
  return `Route changed from ${oldColor ?? "none"} to ${newColor}. Previous search/approach cancelled.`;
}

export function validateRouteChange(newDiceColor: string): { ok: true; color: DiceColor } | { ok: false; reason: string } {
  const color = normalizeDiceColor(newDiceColor);
  if (!color) {
    return {
      ok: false,
      reason: `Invalid dice color: "${newDiceColor}". Must be one of red, blue, yellow, black.`,
    };
  }

  const safety = checkDiceColor(color);
  if (!safety.ok) {
    return {
      ok: false,
      reason: safety.reason ?? `Invalid dice color: ${newDiceColor}`,
    };
  }

  return { ok: true, color };
}

export function describeMission3Plan(state: Mission3State): string {
  return [
    `Current target: ${state.activeColor ?? "none"}`,
    `Phase: ${state.phase}`,
    state.lastKnownPose ? `Last pose: (${state.lastKnownPose.x}, ${state.lastKnownPose.y})` : "No pose known yet",
  ].join(" | ");
}

export function updateMission3Pose(
  state: Mission3State,
  x: number,
  y: number,
): Mission3State {
  return {
    ...state,
    lastKnownPose: { x, y },
    lastUpdatedAt: Date.now(),
  };
}
