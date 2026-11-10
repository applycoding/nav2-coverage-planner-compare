// SPDX-License-Identifier: Apache-2.0
import { defineConfig } from "@playwright/test";

const baseURL = "http://localhost:4173/nav2-coverage-planner-compare/";

export default defineConfig({
  testDir: "e2e",
  timeout: 120_000,
  use: {
    baseURL,
    channel: "chromium",
  },
  webServer: {
    command: "npm run build && npm run preview",
    port: 4173,
    reuseExistingServer: !process.env.CI,
  },
});
