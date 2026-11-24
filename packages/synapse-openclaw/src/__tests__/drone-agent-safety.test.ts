import assert from "node:assert/strict";
import test from "node:test";
import { parseConfig } from "@corebrum/core";
import type { AgentTool, OpenClawPluginApi } from "../plugin-api.js";
import { registerTools } from "../tools/index.js";
import { registerSafetyHook } from "../safety/validator.js";

const droneConfig = parseConfig({
  transport: { mode: "local" },
  robots: [{ id: "uav", name: "UAV", namespace: "uav", kind: "drone", default: true }],
});

function registeredTools(): Map<string, AgentTool> {
  const tools = new Map<string, AgentTool>();
  const api = {
    logger: { info: () => {}, warn: () => {}, error: () => {} },
    registerTool: (tool: AgentTool) => tools.set(tool.name, tool),
  } as unknown as OpenClawPluginApi;
  registerTools(api, droneConfig);
  return tools;
}

test("openclaw AR-2 refuses generic drone control before contacting ROS", async () => {
  const tools = registeredTools();
  const publish = tools.get("ros2_publish");
  const serviceCall = tools.get("ros2_service_call");
  assert.ok(publish);
  assert.ok(serviceCall);

  const blockedCalls = [
    publish.execute("setpoint", {
      robot_id: "uav",
      topic: "/mavros/setpoint_raw/local",
      type: "mavros_msgs/msg/PositionTarget",
      message: {},
    }),
    publish.execute("cmd-vel", {
      robot_id: "uav",
      topic: "/cmd_vel",
      type: "geometry_msgs/msg/Twist",
      message: {},
    }),
    serviceCall.execute("arming", { robot_id: "uav", service: "/mavros/cmd/arming" }),
    serviceCall.execute("other-service", { robot_id: "uav", service: "/other/service" }),
  ];

  for (const result of await Promise.all(blockedCalls)) {
    const text = result.content[0]?.type === "text" ? result.content[0].text : "";
    assert.match(text, /AR-2/);
    assert.equal((result.details as { success?: boolean } | undefined)?.success, false);
  }
});

test("openclaw AR-2 hook blocks ROS tools registered outside the core wrapper", async () => {
  let beforeToolCall:
    | ((event: { toolName: string; params: Record<string, unknown> }, ctx: { toolName: string }) => Promise<unknown>)
    | undefined;
  const api = {
    logger: { info: () => {}, warn: () => {}, error: () => {} },
    on: (_name: string, handler: typeof beforeToolCall) => {
      beforeToolCall = handler;
    },
  } as unknown as OpenClawPluginApi;
  registerSafetyHook(api, droneConfig);
  assert.ok(beforeToolCall);

  const result = await beforeToolCall(
    { toolName: "ros2_publish", params: { topic: "/mavros/setpoint_raw/local" } },
    { toolName: "ros2_publish" },
  ) as { block?: boolean; blockReason?: string };
  assert.equal(result.block, true);
  assert.match(result.blockReason ?? "", /AR-2/);
});