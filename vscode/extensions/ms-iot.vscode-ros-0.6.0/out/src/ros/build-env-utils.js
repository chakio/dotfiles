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
const path = require("path");
const vscode = require("vscode");
const extension = require("../extension");
const pfs = require("../promise-fs");
const telemetry = require("../telemetry-helper");
const ros_1 = require("./ros");
const PYTHON_AUTOCOMPLETE_PATHS = "python.autoComplete.extraPaths";
/**
 * Creates config files which don't exist.
 */
function createConfigFiles() {
    return __awaiter(this, void 0, void 0, function* () {
        const config = vscode.workspace.getConfiguration();
        // Update the Python path if required.
        if (config.get(PYTHON_AUTOCOMPLETE_PATHS, []).length === 0) {
            updatePythonPathInternal();
        }
        // Ensure the ".vscode" directory exists then update the C++ path.
        const dir = path.join(vscode.workspace.rootPath, ".vscode");
        if (!(yield pfs.exists(dir))) {
            yield pfs.mkdir(dir);
        }
        pfs.exists(path.join(dir, "c_cpp_properties.json")).then(exists => {
            if (!exists) {
                updateCppPropertiesInternal();
            }
        });
    });
}
exports.createConfigFiles = createConfigFiles;
function updateCppProperties(context) {
    return __awaiter(this, void 0, void 0, function* () {
        const reporter = telemetry.getReporter(context);
        reporter.sendTelemetryCommand(extension.Commands.UpdateCppProperties);
        updateCppPropertiesInternal();
    });
}
exports.updateCppProperties = updateCppProperties;
/**
 * Updates the `c_cpp_properties.json` file with ROS include paths.
 */
function updateCppPropertiesInternal() {
    return __awaiter(this, void 0, void 0, function* () {
        let includes = yield ros_1.rosApi.getIncludeDirs();
        const workspaceIncludes = yield ros_1.rosApi.getWorkspaceIncludeDirs(extension.baseDir);
        includes = includes.concat(workspaceIncludes);
        if (process.platform !== "win32") {
            includes.push(path.join("/", "usr", "include"));
        }
        // append ** so the IntelliSense engine will do a recursive search for hearder files starting from that directory
        includes = includes.map((include) => {
            return path.join(include, "**");
        });
        // https://github.com/Microsoft/vscode-cpptools/blob/master/Documentation/LanguageServer/c_cpp_properties.json.md
        const cppProperties = {
            configurations: [
                {
                    browse: {
                        databaseFilename: "",
                        limitSymbolsToIncludedHeaders: true,
                    },
                    includePath: includes,
                    name: "ROS",
                },
            ],
        };
        const filename = path.join(vscode.workspace.rootPath, ".vscode", "c_cpp_properties.json");
        yield pfs.writeFile(filename, JSON.stringify(cppProperties, undefined, 2));
    });
}
function updatePythonPath(context) {
    const reporter = telemetry.getReporter(context);
    reporter.sendTelemetryCommand(extension.Commands.UpdatePythonPath);
    updatePythonPathInternal();
}
exports.updatePythonPath = updatePythonPath;
/**
 * Updates the python autocomplete path to support ROS.
 */
function updatePythonPathInternal() {
    vscode.workspace.getConfiguration().update(PYTHON_AUTOCOMPLETE_PATHS, extension.env.PYTHONPATH.split(path.delimiter));
}
//# sourceMappingURL=build-env-utils.js.map