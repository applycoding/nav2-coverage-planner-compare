// SPDX-License-Identifier: Apache-2.0
import { mountApp } from "./app";

const root = document.querySelector<HTMLDivElement>("#app");
if (root) mountApp(root);
