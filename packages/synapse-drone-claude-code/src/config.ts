import fs from "node:fs";
import path from "node:path";
import os from "node:os";
import type { SynapseConfig } from "@corebrum/core";
import { parseConfig } from "@corebrum/core";

/**
 * Override robot namespace without editing JSON.
 * Set in .mcp.json env block or shell: SYNAPSE_ROBOT_NAMESPACE=<ns>
 */
function applyEnvOverrides(config: SynapseConfig): SynapseConfig {
  const ns = process.env.SYNAPSE_ROBOT_NAMESPACE?.trim();
  if (!ns) return config;
  return { ...config, robot: { ...config.robot, namespace: ns } };
}

function getConfigPath(): string {
  const env = process.env.SYNAPSE_CONFIG_PATH;
  if (env?.trim()) return path.resolve(env);
  return path.join(os.homedir(), ".synapse", "config.json");
}

function loadRaw(): Record<string, unknown> {
  const p = getConfigPath();
  try {
    const raw = fs.readFileSync(p, "utf8");
    const parsed = JSON.parse(raw) as unknown;
    if (parsed !== null && typeof parsed === "object") {
      process.stderr.write(`[Drone MCP] Config from ${p}\n`);
      return parsed as Record<string, unknown>;
    }
  } catch (err) {
    const nodeErr = err as NodeJS.ErrnoException;
    if (nodeErr.code !== "ENOENT") throw err;
    throw new Error(
      `Drone MCP config not found at ${p}. ` +
      "Create ~/.synapse/config.json or set SYNAPSE_CONFIG_PATH.",
    );
  }
  return {};
}

export function loadConfig(): SynapseConfig {
  return applyEnvOverrides(parseConfig(loadRaw()));
}

/** Downward camera topic — set via env or falls back to ICD default. */
export function downwardCameraTopic(config: SynapseConfig): string {
  // TODO: ICD N3 — confirm downward camera topic name once signed (target: Nov 6)
  return (
    process.env.SYNAPSE_DRONE_DOWNWARD_CAMERA?.trim() ??
    (config.robot as Record<string, unknown> | undefined)?.cameraTopic as string | undefined ??
    "/camera/downward/color/image_raw/compressed"
  );
}

/** Forward-facing camera topic. */
export function forwardCameraTopic(): string {
  // TODO: ICD N3 — confirm forward camera topic name once signed
  return (
    process.env.SYNAPSE_DRONE_FORWARD_CAMERA?.trim() ??
    "/camera/forward/color/image_raw/compressed"
  );
}
