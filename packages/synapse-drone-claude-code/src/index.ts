#!/usr/bin/env node
/**
 * UAV-ND AgenticROS drone MCP server.
 *
 * Exposes 13 coarse drone verbs as MCP tools over stdio.
 * The agent never publishes raw velocities — it calls action/service verbs
 * that route through the ROS 2 safety supervisor (N11).
 *
 * Register with Claude Code:
 *   claude mcp add --transport stdio drone -- node /path/to/dist/index.js
 *
 * Or add to .mcp.json:
 *   { "mcpServers": { "drone": { "type": "stdio",
 *       "command": "sh",
 *       "args": ["-c", "node packages/synapse-drone-claude-code/dist/index.js 2>>/tmp/drone-mcp.log"] } } }
 *
 * stdout is reserved for MCP JSON-RPC; all logs go to stderr.
 */

import { format } from "util";
import { Server } from "@modelcontextprotocol/sdk/server/index.js";
import { StdioServerTransport } from "@modelcontextprotocol/sdk/server/stdio.js";
import { ListToolsRequestSchema, CallToolRequestSchema } from "@modelcontextprotocol/sdk/types.js";
import type { SynapseConfig } from "@corebrum/core";
import { loadConfig } from "./config.js";
import { connect, disconnect } from "./transport.js";
import { DRONE_TOOLS, handleDroneToolCall } from "./tools.js";

let config: SynapseConfig | null = null;
let transportConnected = false;

async function ensureConnected(): Promise<void> {
  if (transportConnected) return;
  if (!config) config = loadConfig();
  await connect(config);
  transportConnected = true;
}

function main(): void {
  // Avoid crash on EPIPE when Claude Code disconnects
  process.stdout.on("error", (err: NodeJS.ErrnoException) => {
    if (err?.code !== "EPIPE") throw err;
  });
  process.stderr.on("error", (err: NodeJS.ErrnoException) => {
    if (err?.code !== "EPIPE") throw err;
  });

  // Keep stdout clean for MCP; redirect log/info to stderr
  const toStderr = (args: unknown[]) => process.stderr.write(format(...args) + "\n");
  console.log = (...args: unknown[]) => toStderr(args);
  console.info = (...args: unknown[]) => toStderr(args);

  process.stderr.write("[UAV-ND Drone MCP] Starting — 13 drone verb tools loaded\n");

  const server = new Server(
    { name: "drone", version: "0.1.0" },
    { capabilities: { tools: {} } },
  );

  server.setRequestHandler(ListToolsRequestSchema, () => ({
    tools: DRONE_TOOLS.map((t) => ({
      name: t.name,
      description: t.description,
      inputSchema: t.inputSchema,
    })),
  }));

  server.setRequestHandler(CallToolRequestSchema, async (request) => {
    const { name, arguments: args } = request.params;
    try {
      config = loadConfig();
      await ensureConnected();
      const result = await handleDroneToolCall(name, args ?? {}, config);
      return { content: result.content, isError: result.isError };
    } catch (error) {
      const message = error instanceof Error ? error.message : String(error);
      return {
        content: [{ type: "text", text: `Tool error: ${message}` }],
        isError: true,
      };
    }
  });

  const transport = new StdioServerTransport();

  async function run(): Promise<void> {
    await server.connect(transport);
  }

  function shutdown(): void {
    disconnect()
      .then(() => process.exit(0))
      .catch(() => process.exit(1));
  }

  process.on("SIGINT", shutdown);
  process.on("SIGTERM", shutdown);

  run().catch((error) => {
    process.stderr.write(`[UAV-ND Drone MCP] Fatal: ${error}\n`);
    process.exit(1);
  });
}

main();
