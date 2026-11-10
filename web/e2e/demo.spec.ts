// SPDX-License-Identifier: Apache-2.0
import { mkdirSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";
import { expect, test } from "@playwright/test";

test("office A* and hole coverage demo", async ({ page }) => {
  const errors: string[] = [];
  page.on("console", (msg) => {
    if (msg.type() === "error") errors.push(msg.text());
  });

  await page.goto("./");
  await page.locator("#mapSelect").selectOption("office");
  await page.locator("#planner").selectOption("astar");
  await page.locator("#planBtn").click();
  const metrics = page.locator('[data-testid="metrics"]');
  await expect(metrics).toContainText("success length=", { timeout: 30_000 });
  await expect(metrics).toContainText("length=");

  await page.locator("#mapSelect").selectOption("hole");
  await page.locator("#coverageBtn").click();
  await expect(metrics).toContainText("success length=", { timeout: 60_000 });
  await expect(metrics).not.toContainText("failed:");
  await expect(metrics).not.toContainText("partial_coverage");

  expect(errors).toEqual([]);

  const repoRoot = join(dirname(fileURLToPath(import.meta.url)), "../..");
  const docsDir = join(repoRoot, "docs");
  mkdirSync(docsDir, { recursive: true });
  await page.screenshot({ path: join(docsDir, "demo.png"), fullPage: true });
});
