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
const path = require("path");
const vscode = require("vscode");
const extension = require("../extension");
const telemetry = require("../telemetry-helper");
const ros_1 = require("./ros");
function rosrun(context) {
    return __awaiter(this, void 0, void 0, function* () {
        const reporter = telemetry.getReporter(context);
        reporter.sendTelemetryCommand(extension.Commands.Rosrun);
        const terminal = yield preparerosrun();
        terminal.show();
    });
}
exports.rosrun = rosrun;
function preparerosrun() {
    return __awaiter(this, void 0, void 0, function* () {
        const packageName = yield vscode.window.showQuickPick(ros_1.rosApi.getPackageNames(), {
            placeHolder: "Choose a package",
        });
        if (!packageName) {
            return;
        }
        let basenames = (files) => files.map((file) => path.basename(file));
        const executables = ros_1.rosApi.findPackageExecutables(packageName).then(basenames);
        let target = yield vscode.window.showQuickPick(executables, { placeHolder: "Choose an executable" });
        if (!target) {
            return;
        }
        let argument = yield vscode.window.showInputBox({ placeHolder: "Enter any extra arguments" });
        if (argument == undefined) {
            return;
        }
        return ros_1.rosApi.activateRosrun(packageName, target, argument);
    });
}
function roslaunch(context) {
    return __awaiter(this, void 0, void 0, function* () {
        const reporter = telemetry.getReporter(context);
        reporter.sendTelemetryCommand(extension.Commands.Roslaunch);
        let terminal = yield prepareroslaunch();
        terminal.show();
    });
}
exports.roslaunch = roslaunch;
function prepareroslaunch() {
    return __awaiter(this, void 0, void 0, function* () {
        const packageName = yield vscode.window.showQuickPick(ros_1.rosApi.getPackageNames(), {
            placeHolder: "Choose a package",
        });
        if (!packageName) {
            return;
        }
        const launchFiles = yield ros_1.rosApi.findPackageLaunchFiles(packageName);
        const launchFileBasenames = launchFiles.map((filename) => path.basename(filename));
        let target = yield vscode.window.showQuickPick(launchFileBasenames, { placeHolder: "Choose a launch file" });
        const launchFilePath = launchFiles[launchFileBasenames.indexOf(target)];
        if (!launchFilePath) {
            return;
        }
        let argument = yield vscode.window.showInputBox({ placeHolder: "Enter any extra arguments" });
        if (argument == undefined) {
            return;
        }
        return ros_1.rosApi.activateRoslaunch(launchFilePath, argument);
    });
}
//# sourceMappingURL=cli.js.map