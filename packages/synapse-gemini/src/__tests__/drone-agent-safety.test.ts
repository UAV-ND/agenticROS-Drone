import assert from "node:assert/strict";
import test from "node:test";
import { parseConfig } from "@corebrum/core";
import { executeTool } from "../tools.js";

const droneConfig = parseConfig({
  transport: { mode: "local" },
  robots: [{ id: "uav", name: "UAV", namespace: "uav", kind: "drone", default: true }],
});

test("gemini AR-2 refuses generic drone control before contacting ROS", async () => {
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
  ];

  for (const attempt of attempts) {
    const result = await executeTool(attempt.name, attempt.args, droneConfig);
    assert.match(result.output, /AR-2/);
  }
});