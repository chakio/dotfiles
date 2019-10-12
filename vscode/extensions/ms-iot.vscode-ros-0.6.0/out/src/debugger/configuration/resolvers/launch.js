"use strict";
// Copyright (c) Andrew Short. All rights reserved.
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
const fs = require("fs");
const yaml = require("js-yaml");
const os = require("os");
const path = require("path");
const shell_quote = require("shell-quote");
const tmp = require("tmp");
const util = require("util");
const vscode = require("vscode");
const extension = require("../../../extension");
const promisifiedExec = util.promisify(child_process.exec);
class LaunchResolver {
    // tslint:disable-next-line: max-line-length
    resolveDebugConfiguration(folder, config, token) {
        return __awaiter(this, void 0, void 0, function* () {
            if (!path.isAbsolute(config.target) || path.extname(config.target) !== ".launch") {
                throw new Error("Launch request requires an absolute path as target.");
            }
            const rosExecOptions = {
                env: extension.env,
            };
            let result = yield promisifiedExec(`roslaunch --dump-params ${config.target}`, rosExecOptions);
            const parameters = Object.keys(yaml.load(result.stdout));
            if (parameters && parameters.length) {
                // only call into rosparam when necessary
                const tmpFile = tmp.fileSync();
                fs.writeFile(`${tmpFile.name}`, result.stdout, (error) => __awaiter(this, void 0, void 0, function* () {
                    if (error) {
                        throw error;
                    }
                    yield promisifiedExec(`rosparam load ${tmpFile.name}`, rosExecOptions);
                    tmpFile.removeCallback();
                }));
            }
            result = yield promisifiedExec(`roslaunch --nodes ${config.target}`, rosExecOptions);
            const nodes = result.stdout.trim().split(os.EOL);
            yield Promise.all(nodes.map((node) => {
                return promisifiedExec(`roslaunch --args ${node} ${config.target}`, rosExecOptions);
            })).then((commands) => {
                for (const command of commands) {
                    const roslaunchRequest = this.parseRoslaunchCommand(command.stdout);
                    this.executeRoslaunchRequest(roslaunchRequest);
                }
            });
            return config;
        });
    }
    parseRoslaunchCommand(command) {
        // escape backslash in file path
        const parsedArgs = shell_quote.parse(command.replace(/[\\]/g, "\\$&"));
        const envConfig = {};
        while (parsedArgs) {
            if (parsedArgs[0].toString().includes("=")) {
                const arg = parsedArgs.shift().toString();
                envConfig[arg.substring(0, arg.indexOf("="))] = arg.substring(arg.indexOf("=") + 1);
            }
            else {
                break;
            }
        }
        const request = {
            executable: parsedArgs.shift().toString(),
            arguments: parsedArgs.map((arg) => {
                return arg.toString();
            }),
            cwd: ".",
            env: Object.assign({}, extension.env, envConfig),
        };
        return request;
    }
    executeRoslaunchRequest(request) {
        return __awaiter(this, void 0, void 0, function* () {
            request.executable = request.executable.toLowerCase();
            if (request.executable.endsWith("python") || request.executable.endsWith("python.exe")) {
                const pythonScript = request.arguments.shift();
                const pythonlaunchdebugconfiguration = {
                    name: `Python: launch`,
                    type: "python",
                    request: "launch",
                    program: pythonScript,
                    args: request.arguments,
                    env: request.env,
                    stopOnEntry: true,
                };
                const launched = yield vscode.debug.startDebugging(undefined, pythonlaunchdebugconfiguration);
                if (!launched) {
                    throw (new Error(`Failed to start debug session!`));
                }
            }
            else if (os.platform() === "win32" && request.executable.endsWith(".exe")) {
                const envConfigs = [];
                for (const key in request.env) {
                    if (request.env.hasOwnProperty(key)) {
                        envConfigs.push({
                            name: key,
                            value: request.env[key],
                        });
                    }
                }
                const cpplaunchdebugconfiguration = {
                    name: "C++: launch",
                    type: "cppvsdbg",
                    request: "launch",
                    cwd: ".",
                    program: request.executable,
                    args: request.arguments,
                    environment: envConfigs,
                    stopAtEntry: true,
                };
                const launched = yield vscode.debug.startDebugging(undefined, cpplaunchdebugconfiguration);
                if (!launched) {
                    throw (new Error(`Failed to start debug session!`));
                }
            }
        });
    }
}
exports.LaunchResolver = LaunchResolver;
//# sourceMappingURL=launch.js.map