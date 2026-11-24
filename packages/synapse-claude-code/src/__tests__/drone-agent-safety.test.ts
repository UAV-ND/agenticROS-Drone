import assert from "node:assert/strict";
import test from "node:test";
import { parseConfig } from "@corebrum/core";
import { handleToolCall } from "../tools.js";

const droneConfig = parseConfig({
  transport: { mode: "local" },
  robots: [{ id: "uav", name: "UAV", namespace: "uav", kind: "drone", default: true }],
});

test("mcp AR-2 refuses generic drone control before contacting ROS", async () => {
  const attempts = [
    {
      name: "ros2_publish",
      args: {
        topic: "/mavros/setpoint_raw/local",
        type: "mavros_msgs/msg/PositionTarget",
        message: {},
      },
    },
    { name: "ros2_publish", args: { topic: "/cmd_vel", type: "geometry_msgs/msg/Twist", message: {} } },
    { name: "ros2_publish", args: { topic: "/uav/die/target", type: "geometry_msgs/msg/PointStamped", message: {} } },
    { name: "ros2_service_call", args: { service: "/mavros/cmd/arming" } },
    { name: "ros2_service_call", args: { service: "/other/service" } },
    { name: "ros2_action_goal", args: { action: "/navigate", actionType: "test/action/Go", goal: {} } },
    { name: "ros2_param_set", args: { node: "/mavros", parameter: "value", value: 1 } },
    { name: "run_mission", args: {} },
  ];

  for (const attempt of attempts) {
    const result = await handleToolCall(attempt.name, attempt.args, droneConfig);
    assert.equal(result.isError, true, `${attempt.name} should be refused`);
    assert.match(result.content[0]?.type === "text" ? result.content[0].text : "", /AR-2/);
  }
});