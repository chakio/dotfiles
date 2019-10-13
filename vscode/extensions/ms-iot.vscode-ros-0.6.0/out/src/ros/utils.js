"use strict";
// Copyright (c) Andrew Short. All rights reserved.
// Licensed under the MIT License.
Object.defineProperty(exports, "__esModule", { value: true });
const child_process = require("child_process");
const os = require("os");
const vscode = require("vscode");
const extension = require("../extension");
const pfs = require("../promise-fs");
const telemetry = require("../telemetry-helper");
/**
 * Executes a setup file and returns the resulting env.
 */
function sourceSetupFile(filename, env) {
    return new Promise((resolve, reject) => {
        let exportEnvCommand;
        if (process.platform === "win32") {
            exportEnvCommand = `cmd /c "\"${filename}\" && set"`;
        }
        else {
            exportEnvCommand = `bash -c "source '${filename}' && env"`;
        }
        let processOptions = {
            cwd: extension.baseDir,
            env: env,
        };
        child_process.exec(exportEnvCommand, processOptions, (error, stdout, _stderr) => {
            if (!error) {
                resolve(stdout.split(os.EOL).reduce((env, line) => {
                    const index = line.indexOf("=");
                    if (index !== -1) {
                        env[line.substr(0, index)] = line.substr(index + 1);
                    }
                    return env;
                }, {}));
            }
            else {
                reject(error);
            }
        });
    });
}
exports.sourceSetupFile = sourceSetupFile;
function xacro(filename) {
    return new Promise((resolve, reject) => {
        let processOptions = {
            cwd: extension.baseDir,
            env: extension.env,
            windowsHide: false,
        };
        let xacroCommand;
        if (process.platform === "win32") {
            xacroCommand = `cmd /c "xacro "${filename}""`;
        }
        else {
            xacroCommand = `bash -c "xacro '${filename}' && env"`;
        }
        child_process.exec(xacroCommand, processOptions, (error, stdout, _stderr) => {
            if (!error) {
                resolve(stdout);
            }
            else {
                reject(error);
            }
        });
    });
}
exports.xacro = xacro;
/**
 * Gets the names of installed distros.
 */
function getDistros() {
    return pfs.readdir("/opt/ros");
}
exports.getDistros = getDistros;
/**
 * Creates and shows a ROS-sourced terminal.
 */
function createTerminal(context) {
    const reporter = telemetry.getReporter(context);
    reporter.sendTelemetryCommand(extension.Commands.CreateTerminal);
    vscode.window.createTerminal({ name: 'ROS', env: extension.env }).show();
}
exports.createTerminal = createTerminal;
//# sourceMappingURL=utils.js.map