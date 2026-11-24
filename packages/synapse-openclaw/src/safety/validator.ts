import type { OpenClawPluginApi } from "../plugin-api.js";
import type { SynapseConfig } from "@corebrum/core";
import {
  checkActionGoalSafety,
  checkDroneAgentToolSafety,
  checkPublishSafety,
  resolveRobotFromArgs,
} from "@corebrum/core";

/**
 * Register the before_tool_call safety validation hook.
 * Intercepts Twist publishes and navigation goals against per-robot limits.
 */
export function registerSafetyHook(api: OpenClawPluginApi, config: SynapseConfig): void {
  api.on("before_tool_call", async (event, _ctx) => {
    let robot;
    try {
      robot = resolveRobotFromArgs(config, event.params);
    } catch {
      robot = undefined;
    }

    if (!robot && (event.toolName.startsWith("ros2_") || event.toolName === "run_mission")) {
      const blockReason = "AR-2: ROS tool call denied because the target robot could not be resolved.";
      api.logger.warn(`Blocked: ${blockReason}`);
      return { block: true, blockReason };
    }

    const agentSafety = checkDroneAgentToolSafety(event.toolName, event.params, robot);
    if (agentSafety.block) {
      api.logger.warn(`Blocked: ${agentSafety.blockReason}`);
      return { block: true, blockReason: agentSafety.blockReason };
    }

    if (event.toolName === "ros2_publish") {
      const safe = checkPublishSafety(config, event.params, robot);
      if (safe.block) {
        api.logger.warn(`Blocked: ${safe.blockReason}`);
        return { block: true, blockReason: safe.blockReason };
      }
    }

    if (event.toolName === "ros2_action_goal") {
      const safe = checkActionGoalSafety(config, event.params, robot);
      if (safe.block) {
        api.logger.warn(`Blocked: ${safe.blockReason}`);
        return { block: true, blockReason: safe.blockReason };
      }
    }

    return undefined;
  });
}
