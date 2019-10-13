"use strict";
// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.
var __awaiter = (this && this.__awaiter) || function (thisArg, _arguments, P, generator) {
    return new (P || (P = Promise))(function (resolve, reject) {
        function fulfilled(value) { try { step(generator.next(value)); } catch (e) { reject(e); } }
        function rejected(value) { try { step(generator["throw"](value)); } catch (e) { reject(e); } }
        function step(result) { result.done ? resolve(result.value) : new P(function (resolve) { resolve(result.value); }).then(fulfilled, rejected); }
        step((generator = generator.apply(thisArg, _arguments || [])).next());
    });
};
Object.defineProperty(exports, "__esModule", { value: true });
const child_process = require("child_process");
const util = require("util");
const vscode = require("vscode");
const extension = require("../../extension");
const ros2_monitor = require("./ros2-monitor");
/**
 * start the ROS2 daemon.
 */
function startDaemon() {
    return __awaiter(this, void 0, void 0, function* () {
        const command = "ros2 daemon start";
        const exec = util.promisify(child_process.exec);
        yield exec(command, { env: this.env });
    });
}
exports.startDaemon = startDaemon;
/**
 * stop the ROS2 daemon.
 */
function stopDaemon() {
    return __awaiter(this, void 0, void 0, function* () {
        const command = "ros2 daemon stop";
        const exec = util.promisify(child_process.exec);
        yield exec(command, { env: this.env });
    });
}
exports.stopDaemon = stopDaemon;
/**
 * Shows the ROS core status in the status bar.
 */
class StatusBarItem {
    constructor() {
        this.item = vscode.window.createStatusBarItem(vscode.StatusBarAlignment.Left, 200);
        this.item.text = "$(question) ROS2 Daemon";
        this.item.command = extension.Commands.ShowCoreStatus;
        this.ros2cli = new ros2_monitor.XmlRpcApi();
    }
    activate() {
        this.item.show();
        this.timeout = setTimeout(() => this.update(), 200);
    }
    dispose() {
        clearTimeout(this.timeout);
        this.item.dispose();
    }
    update() {
        return __awaiter(this, void 0, void 0, function* () {
            let status = false;
            try {
                const result = yield this.ros2cli.getNodeNamesAndNamespaces();
                status = true;
            }
            catch (error) {
                // do nothing.
            }
            finally {
                this.item.text = (status ? "$(check)" : "$(x)") + " ROS2 Daemon";
                this.timeout = setTimeout(() => this.update(), 200);
            }
        });
    }
}
exports.StatusBarItem = StatusBarItem;
//# sourceMappingURL=daemon.js.map