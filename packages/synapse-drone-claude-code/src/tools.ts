/**
 * UAV-ND AgenticROS drone MCP tool definitions and handlers.
 *
 * Design rules (from sprint board):
 *   - NO raw velocity publish. Every verb maps to a ROS 2 action or service.
 *   - Coarse verbs only; each is its own tool with simple inputs.
 *   - All action/service names marked TODO: ICD N3 — update once signed (target Nov 6).
 *   - Every tool returns a plain-text result the LLM can relay verbatim.
 *
 * Tools by challenge:
 *   Challenge 1 & 2: drone_takeoff, drone_navigate, drone_search,
 *                    drone_detect_dice, drone_center_and_hover,
 *                    drone_pick_up, drone_return_home
 *   Challenge 3 adds: drone_change_route
 *   All challenges:  drone_abort, drone_get_status, drone_take_snapshot,
 *                    drone_set_keepout, drone_set_search_hint
 */

import type { SynapseConfig } from "@corebrum/core";
import type { Tool } from "@modelcontextprotocol/sdk/types.js";
import {
  cameraSnapshotFromPlainMessage,
  mimeTypeForSnapshotBase64,
} from "@corebrum/ros-camera";
import { getTransport } from "./transport.js";
import { downwardCameraTopic, forwardCameraTopic } from "./config.js";
import {
  checkTakeoffAltitude,
  checkDiceColor,
  checkNavTarget,
  DEFAULT_TAKEOFF_ALTITUDE_M,
} from "./safety.js";

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

function ok(text: string): ToolResult {
  return { content: [{ type: "text", text }] };
}

function err(text: string): ToolResult {
  return { content: [{ type: "text", text }], isError: true };
}

interface ToolResult {
  content: Array<{ type: "text"; text: string } | { type: "image"; data: string; mimeType: string }>;
  isError?: boolean;
}

/** Subscribe to a topic and return the first message within timeoutMs. */
async function subscribeOnce(
  transport: Awaited<ReturnType<typeof getTransport>>,
  topic: string,
  type: string,
  timeoutMs = 5_000,
): Promise<Record<string, unknown>> {
  return new Promise((resolve, reject) => {
    const timer = setTimeout(() => {
      sub.unsubscribe();
      reject(new Error(`Timeout waiting for message on ${topic}`));
    }, timeoutMs);

    const sub = transport.subscribe({ topic, type }, (msg) => {
      clearTimeout(timer);
      sub.unsubscribe();
      resolve(msg);
    });
  });
}

// ---------------------------------------------------------------------------
// Tool definitions (MCP schema)
// ---------------------------------------------------------------------------

export const DRONE_TOOLS: Tool[] = [
  // ---- Challenge 1/2/3 ----

  {
    name: "drone_takeoff",
    description:
      "Arm the drone and take off to the specified altitude. Default altitude is 3 m (competition standard). " +
      "Returns when the drone reaches the target altitude and hovers.",
    inputSchema: {
      type: "object",
      properties: {
        altitude_m: {
          type: "number",
          description: `Target altitude in metres above the launch point. Default: ${DEFAULT_TAKEOFF_ALTITUDE_M} m.`,
        },
      },
    },
  },

  {
    name: "drone_navigate",
    description:
      "Fly to a 2-D map position (x, y in the odom/map frame, TBD in ICD N3). " +
      "Optionally override altitude. Returns when the drone reaches the waypoint.",
    inputSchema: {
      type: "object",
      properties: {
        x: { type: "number", description: "Target x position in metres (map frame)." },
        y: { type: "number", description: "Target y position in metres (map frame)." },
        altitude_m: { type: "number", description: "Hold altitude in metres (optional; keeps current if omitted)." },
      },
      required: ["x", "y"],
    },
  },

  {
    name: "drone_search",
    description:
      "Execute a search pattern (lawnmower or expanding-square) to find dice in the field. " +
      "Optionally constrain the search to a specific target color. " +
      "Returns a summary of detected dice or 'no dice found' if the pattern completes without a detection.",
    inputSchema: {
      type: "object",
      properties: {
        target_color: {
          type: "string",
          enum: ["red", "blue", "yellow", "black"],
          description: "Only return detections of this color. Omit to report any color.",
        },
        search_radius_m: {
          type: "number",
          description: "Radius of the search area in metres around the current position. Default set by GNC.",
        },
      },
    },
  },

  {
    name: "drone_detect_dice",
    description:
      "Query the CV world model for currently visible dice. Does not move the drone. " +
      "Returns a list of detected dice with color, confidence, and estimated position.",
    inputSchema: {
      type: "object",
      properties: {
        target_color: {
          type: "string",
          enum: ["red", "blue", "yellow", "black"],
          description: "Filter results to this color only. Omit to return all detected dice.",
        },
      },
    },
  },

  {
    name: "drone_center_and_hover",
    description:
      "Fly directly above the specified die and hold a stable hover for pickup. " +
      "The drone centers using the downward camera and CV. Returns when centered and stable.",
    inputSchema: {
      type: "object",
      properties: {
        target_color: {
          type: "string",
          enum: ["red", "blue", "yellow", "black"],
          description: "Color of the target die to center on.",
        },
      },
      required: ["target_color"],
    },
  },

  {
    name: "drone_pick_up",
    description:
      "Trigger the pickup mechanism to retrieve the die the drone is currently centered over. " +
      "The drone must be in a centered hover (use drone_center_and_hover first). " +
      "Returns success or failure with reason.",
    inputSchema: {
      type: "object",
      properties: {},
    },
  },

  {
    name: "drone_return_home",
    description:
      "Return to the launch point and land inside the 5-ft start square. " +
      "Equivalent to RTL in ArduPilot. Returns when the drone has landed.",
    inputSchema: {
      type: "object",
      properties: {},
    },
  },

  // ---- Challenge 3 only ----

  {
    name: "drone_change_route",
    description:
      "Update the mission target to a new die color mid-flight (Challenge 3 mid-mission target change). " +
      "Cancels any in-progress search or approach for the old target.",
    inputSchema: {
      type: "object",
      properties: {
        new_dice_color: {
          type: "string",
          enum: ["red", "blue", "yellow", "black"],
          description: "The new target die color.",
        },
      },
      required: ["new_dice_color"],
    },
  },

  // ---- All challenges ----

  {
    name: "drone_abort",
    description:
      "Emergency abort: immediately cancel all active actions and switch to Return-to-Launch (RTL). " +
      "Use this for safety emergencies, judge interrupts, or mission failures.",
    inputSchema: {
      type: "object",
      properties: {},
    },
  },

  {
    name: "drone_get_status",
    description:
      "Get the current vehicle status: flight mode, armed state, altitude, battery voltage, and any active action.",
    inputSchema: {
      type: "object",
      properties: {},
    },
  },

  {
    name: "drone_take_snapshot",
    description:
      "Capture a camera image. Use 'downward' (default) for the dice-detection camera or 'forward' for the forward-facing camera.",
    inputSchema: {
      type: "object",
      properties: {
        camera: {
          type: "string",
          enum: ["downward", "forward"],
          description: "Which camera to snapshot. Default: downward.",
        },
        timeout_ms: {
          type: "number",
          description: "Max wait for a frame in milliseconds. Default: 5000.",
        },
      },
    },
  },

  {
    name: "drone_set_keepout",
    description:
      "Define a circular keepout zone the drone must not enter. " +
      "Useful for avoiding obstacles (cones, buckets, boxes) in the competition field.",
    inputSchema: {
      type: "object",
      properties: {
        center_x: { type: "number", description: "Zone centre x in metres (map frame)." },
        center_y: { type: "number", description: "Zone centre y in metres (map frame)." },
        radius_m: { type: "number", description: "Exclusion radius in metres." },
        label: { type: "string", description: "Optional human-readable label for this zone." },
      },
      required: ["center_x", "center_y", "radius_m"],
    },
  },

  {
    name: "drone_set_search_hint",
    description:
      "Provide the search planner a location hint for the next search. " +
      "Use when a team member or prior CV detection suggests a likely die location.",
    inputSchema: {
      type: "object",
      properties: {
        x: { type: "number", description: "Hint x in metres (map frame)." },
        y: { type: "number", description: "Hint y in metres (map frame)." },
        confidence: {
          type: "number",
          description: "Hint confidence 0–1 (higher = search here first). Default: 0.5.",
        },
        label: { type: "string", description: "Optional description of why this hint was set." },
      },
      required: ["x", "y"],
    },
  },
];

// ---------------------------------------------------------------------------
// Tool handlers
// ---------------------------------------------------------------------------

export async function handleDroneToolCall(
  name: string,
  args: Record<string, unknown>,
  config: SynapseConfig,
): Promise<ToolResult> {
  const transport = await getTransport(config);

  switch (name) {
    // ------------------------------------------------------------------
    case "drone_takeoff": {
      const altitude_m = typeof args.altitude_m === "number"
        ? args.altitude_m
        : DEFAULT_TAKEOFF_ALTITUDE_M;

      const check = checkTakeoffAltitude(altitude_m);
      if (!check.ok) return err(`Safety check failed: ${check.reason}`);

      // TODO: ICD N3 — update action name + type once signed (target Nov 6)
      const result = await transport.sendActionGoal({
        action: "/drone/takeoff",
        actionType: "uav_msgs/action/Takeoff",
        args: { target_altitude: altitude_m },
      });

      return result.result
        ? ok(`Takeoff complete. Holding at ${altitude_m} m.`)
        : err(`Takeoff failed: ${JSON.stringify(result.values ?? {})}`);
    }

    // ------------------------------------------------------------------
    case "drone_navigate": {
      const x = args.x as number;
      const y = args.y as number;
      const altitude_m = typeof args.altitude_m === "number" ? args.altitude_m : undefined;

      const check = checkNavTarget(x, y, altitude_m);
      if (!check.ok) return err(`Safety check failed: ${check.reason}`);

      // TODO: ICD N3 — update action name, type, and frame conventions once signed
      const result = await transport.sendActionGoal({
        action: "/drone/navigate",
        actionType: "uav_msgs/action/Navigate",
        args: {
          target_x: x,
          target_y: y,
          ...(altitude_m !== undefined ? { target_altitude: altitude_m } : {}),
        },
      });

      const pos = `(${x.toFixed(2)}, ${y.toFixed(2)})${altitude_m !== undefined ? ` @ ${altitude_m} m` : ""}`;
      return result.result
        ? ok(`Reached ${pos}.`)
        : err(`Navigation failed: ${JSON.stringify(result.values ?? {})}`);
    }

    // ------------------------------------------------------------------
    case "drone_search": {
      const target_color = typeof args.target_color === "string" ? args.target_color : undefined;
      const search_radius_m = typeof args.search_radius_m === "number" ? args.search_radius_m : undefined;

      if (target_color) {
        const check = checkDiceColor(target_color);
        if (!check.ok) return err(`Safety check failed: ${check.reason}`);
      }

      // TODO: ICD N3 — update action name + type; C11 world model replaces YOLO find-object (C13)
      const result = await transport.sendActionGoal({
        action: "/drone/search",
        actionType: "uav_msgs/action/Search",
        args: {
          ...(target_color ? { target_color } : {}),
          ...(search_radius_m !== undefined ? { search_radius_m } : {}),
        },
      });

      const detections = result.values?.detections;
      if (!result.result) {
        return err(`Search failed: ${JSON.stringify(result.values ?? {})}`);
      }
      return ok(
        detections
          ? `Search complete. Detections: ${JSON.stringify(detections)}`
          : "Search complete. No dice detected.",
      );
    }

    // ------------------------------------------------------------------
    case "drone_detect_dice": {
      const target_color = typeof args.target_color === "string" ? args.target_color : undefined;

      if (target_color) {
        const check = checkDiceColor(target_color);
        if (!check.ok) return err(`Safety check failed: ${check.reason}`);
      }

      // TODO: ICD N3 — confirm service name; C11 provides the world model
      const result = await transport.callService({
        service: "/drone/detect_dice",
        type: "uav_msgs/srv/DetectDice",
        args: target_color ? { target_color } : {},
      });

      const detections = result.values?.detections;
      if (!result.result) {
        return err(`Detect failed: ${JSON.stringify(result.values ?? {})}`);
      }
      return ok(
        detections
          ? `Detections: ${JSON.stringify(detections)}`
          : "No dice currently detected.",
      );
    }

    // ------------------------------------------------------------------
    case "drone_center_and_hover": {
      const target_color = args.target_color as string;
      const check = checkDiceColor(target_color);
      if (!check.ok) return err(`Safety check failed: ${check.reason}`);

      // TODO: ICD N3 — update action name + type once signed
      const result = await transport.sendActionGoal({
        action: "/drone/center_and_hover",
        actionType: "uav_msgs/action/CenterAndHover",
        args: { target_color },
      });

      return result.result
        ? ok(`Centered and hovering over ${target_color} die. Ready for pickup.`)
        : err(`Center-and-hover failed: ${JSON.stringify(result.values ?? {})}`);
    }

    // ------------------------------------------------------------------
    case "drone_pick_up": {
      // TODO: ICD N3 — update action name + type; confirm pickup mechanism interface with mech team
      const result = await transport.sendActionGoal({
        action: "/drone/pick_up",
        actionType: "uav_msgs/action/PickUp",
        args: {},
      });

      return result.result
        ? ok("Pickup successful. Die secured.")
        : err(`Pickup failed: ${JSON.stringify(result.values ?? {})}`);
    }

    // ------------------------------------------------------------------
    case "drone_return_home": {
      // TODO: ICD N3 — confirm RTL action vs. direct ArduPilot mode change via MAVROS
      const result = await transport.sendActionGoal({
        action: "/drone/return_home",
        actionType: "uav_msgs/action/ReturnHome",
        args: {},
      });

      return result.result
        ? ok("Landed inside start square. Mission complete.")
        : err(`Return home failed: ${JSON.stringify(result.values ?? {})}`);
    }

    // ------------------------------------------------------------------
    case "drone_change_route": {
      const new_dice_color = args.new_dice_color as string;
      const check = checkDiceColor(new_dice_color);
      if (!check.ok) return err(`Safety check failed: ${check.reason}`);

      // TODO: ICD N3 — confirm service vs. action; this may set a param on the GNC node
      const result = await transport.callService({
        service: "/drone/change_route",
        type: "uav_msgs/srv/ChangeRoute",
        args: { new_dice_color },
      });

      return result.result
        ? ok(`Target updated to ${new_dice_color} die. Prior approach/search cancelled.`)
        : err(`Change route failed: ${JSON.stringify(result.values ?? {})}`);
    }

    // ------------------------------------------------------------------
    case "drone_abort": {
      // Cancel any in-progress action, then RTL. This is a best-effort abort
      // at the agent layer; the hardware kill switch is the true hard stop.
      await transport.cancelActionGoal("/drone/takeoff").catch(() => {});
      await transport.cancelActionGoal("/drone/navigate").catch(() => {});
      await transport.cancelActionGoal("/drone/search").catch(() => {});
      await transport.cancelActionGoal("/drone/center_and_hover").catch(() => {});
      await transport.cancelActionGoal("/drone/pick_up").catch(() => {});

      // TODO: ICD N3 — confirm RTL service name (may be MAVROS mode switch)
      const result = await transport.callService({
        service: "/drone/abort",
        type: "uav_msgs/srv/Abort",
        args: {},
      });

      return result.result
        ? ok("Abort acknowledged. Drone returning to launch.")
        : err(`Abort command failed: ${JSON.stringify(result.values ?? {})}. Use the hardware kill switch.`);
    }

    // ------------------------------------------------------------------
    case "drone_get_status": {
      // TODO: ICD N3 — confirm status topic name and message type (mavros_msgs/State + custom?)
      const msg = await subscribeOnce(
        transport,
        "/drone/status",
        "uav_msgs/msg/DroneStatus",
        5_000,
      ).catch(() => null);

      if (!msg) {
        return err("Timed out waiting for drone status. Is the drone node running?");
      }

      const parts: string[] = [];
      if (msg.mode) parts.push(`mode: ${msg.mode}`);
      if (typeof msg.armed === "boolean") parts.push(`armed: ${msg.armed}`);
      if (typeof msg.altitude_m === "number") parts.push(`altitude: ${(msg.altitude_m as number).toFixed(2)} m`);
      if (typeof msg.battery_voltage === "number") parts.push(`battery: ${(msg.battery_voltage as number).toFixed(2)} V`);
      if (msg.active_action) parts.push(`active action: ${msg.active_action}`);

      return ok(parts.length > 0 ? parts.join(" | ") : JSON.stringify(msg));
    }

    // ------------------------------------------------------------------
    case "drone_take_snapshot": {
      const camera = (args.camera as string | undefined) ?? "downward";
      const timeoutMs = typeof args.timeout_ms === "number" ? args.timeout_ms : 5_000;

      const topic =
        camera === "forward" ? forwardCameraTopic() : downwardCameraTopic(config);

      // Determine whether this is a compressed or raw topic
      const isCompressed = topic.endsWith("/compressed");
      const msgType = isCompressed
        ? "sensor_msgs/msg/CompressedImage"
        : "sensor_msgs/msg/Image";

      const msg = await subscribeOnce(transport, topic, msgType, timeoutMs).catch(() => null);
      if (!msg) {
        return err(
          `Timed out waiting for camera frame on ${topic} after ${timeoutMs} ms. ` +
          "Check the camera is publishing and the topic name is correct.",
        );
      }

      const messageKind = isCompressed ? "CompressedImage" : "Image";
      const snapshot = cameraSnapshotFromPlainMessage(messageKind, msg);
      const mimeType = mimeTypeForSnapshotBase64(snapshot.dataBase64, snapshot.formatLabel);
      return {
        content: [
          { type: "image", data: snapshot.dataBase64, mimeType },
          { type: "text", text: `Camera: ${camera} (${topic})` },
        ],
      };
    }

    // ------------------------------------------------------------------
    case "drone_set_keepout": {
      const { center_x, center_y, radius_m, label } = args as {
        center_x: number;
        center_y: number;
        radius_m: number;
        label?: string;
      };

      const check = checkNavTarget(center_x, center_y);
      if (!check.ok) return err(`Safety check failed: ${check.reason}`);
      if (!Number.isFinite(radius_m) || radius_m <= 0) {
        return err(`radius_m must be a positive finite number, got ${radius_m}`);
      }

      // TODO: ICD N3 — confirm service name and message type with GNC (target Nov 6)
      const result = await transport.callService({
        service: "/drone/set_keepout",
        type: "uav_msgs/srv/SetKeeput",
        args: { center_x, center_y, radius_m, ...(label ? { label } : {}) },
      });

      return result.result
        ? ok(`Keepout zone set: centre (${center_x.toFixed(2)}, ${center_y.toFixed(2)}), radius ${radius_m} m${label ? ` [${label}]` : ""}.`)
        : err(`Set keepout failed: ${JSON.stringify(result.values ?? {})}`);
    }

    // ------------------------------------------------------------------
    case "drone_set_search_hint": {
      const x = args.x as number;
      const y = args.y as number;
      const confidence = typeof args.confidence === "number" ? args.confidence : 0.5;
      const label = typeof args.label === "string" ? args.label : undefined;

      const check = checkNavTarget(x, y);
      if (!check.ok) return err(`Safety check failed: ${check.reason}`);
      if (confidence < 0 || confidence > 1) {
        return err(`confidence must be between 0 and 1, got ${confidence}`);
      }

      // TODO: ICD N3 — confirm service name with GNC (target Nov 6)
      const result = await transport.callService({
        service: "/drone/set_search_hint",
        type: "uav_msgs/srv/SetSearchHint",
        args: { x, y, confidence, ...(label ? { label } : {}) },
      });

      return result.result
        ? ok(`Search hint set at (${x.toFixed(2)}, ${y.toFixed(2)}) confidence=${confidence}${label ? ` — ${label}` : ""}.`)
        : err(`Set search hint failed: ${JSON.stringify(result.values ?? {})}`);
    }

    // ------------------------------------------------------------------
    default:
      return err(`Unknown drone tool: ${name}`);
  }
}
