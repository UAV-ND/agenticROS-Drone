import type { SynapseConfig, RosTransport } from "@corebrum/core";
import { TransportPool, resolveRobot } from "@corebrum/core";

let pool = new TransportPool();

/** Return the active robot's transport, lazy-connecting on first call. */
export async function getTransport(config: SynapseConfig): Promise<RosTransport> {
  const robot = resolveRobot(config);
  return pool.acquire(config, robot);
}

/** Pre-warm the active robot's transport at server start. */
export async function connect(config: SynapseConfig): Promise<void> {
  await pool.connectActive(config);
}

/** Drain every connection. Called on SIGINT/SIGTERM. */
export async function disconnect(): Promise<void> {
  await pool.disconnectAll();
}
