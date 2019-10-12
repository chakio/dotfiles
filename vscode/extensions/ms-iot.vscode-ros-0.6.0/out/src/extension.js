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
const cpp_formatter = require("./cpp-formatter");
const pfs = require("./promise-fs");
const telemetry = require("./telemetry-helper");
const vscode_utils = require("./vscode-utils");
const buildtool = require("./build-tool/build-tool");
const ros_build_utils = require("./ros/build-env-utils");
const ros_cli = require("./ros/cli");
const ros_utils = require("./ros/utils");
const ros_1 = require("./ros/ros");
const previewManager_1 = require("./urdfPreview/previewManager");
const debug_manager = require("./debugger/manager");
const debug_utils = require("./debugger/utils");
function setBaseDir(dir) {
    exports.baseDir = dir;
}
exports.setBaseDir = setBaseDir;
let onEnvChanged = new vscode.EventEmitter();
/**
 * Triggered when the env is soured.
 */
exports.onDidChangeEnv = onEnvChanged.event;
/**
 * Subscriptions to dispose when the environment is changed.
 */
let subscriptions = [];
var Commands;
(function (Commands) {
    Commands["CreateCatkinPackage"] = "ros.createCatkinPackage";
    Commands["CreateTerminal"] = "ros.createTerminal";
    Commands["GetDebugSettings"] = "ros.getDebugSettings";
    Commands["Rosrun"] = "ros.rosrun";
    Commands["Roslaunch"] = "ros.roslaunch";
    Commands["ShowCoreStatus"] = "ros.showCoreStatus";
    Commands["StartRosCore"] = "ros.startCore";
    Commands["TerminateRosCore"] = "ros.stopCore";
    Commands["UpdateCppProperties"] = "ros.updateCppProperties";
    Commands["UpdatePythonPath"] = "ros.updatePythonPath";
    Commands["PreviewURDF"] = "ros.previewUrdf";
})(Commands = exports.Commands || (exports.Commands = {}));
function activate(context) {
    return __awaiter(this, void 0, void 0, function* () {
        const reporter = telemetry.getReporter(context);
        exports.extPath = context.extensionPath;
        exports.outputChannel = vscode_utils.createOutputChannel();
        context.subscriptions.push(exports.outputChannel);
        // Activate if we're in a catkin workspace.
        let buildToolDetected = yield buildtool.determineBuildTool(vscode.workspace.rootPath);
        if (!buildToolDetected) {
            return;
        }
        // Activate components when the ROS env is changed.
        context.subscriptions.push(exports.onDidChangeEnv(activateEnvironment.bind(null, context)));
        // Activate components which don't require the ROS env.
        context.subscriptions.push(vscode.languages.registerDocumentFormattingEditProvider("cpp", new cpp_formatter.CppFormatter()));
        previewManager_1.default.INSTANCE.setContext(context);
        // Source the environment, and re-source on config change.
        let config = vscode_utils.getExtensionConfiguration();
        context.subscriptions.push(vscode.workspace.onDidChangeConfiguration(() => {
            const updatedConfig = vscode_utils.getExtensionConfiguration();
            const fields = Object.keys(config).filter(k => !(config[k] instanceof Function));
            const changed = fields.some(key => updatedConfig[key] !== config[key]);
            if (changed) {
                sourceRosAndWorkspace();
            }
            config = updatedConfig;
        }));
        sourceRosAndWorkspace().then(() => {
            vscode.window.registerWebviewPanelSerializer('urdfPreview', previewManager_1.default.INSTANCE);
        });
        reporter.sendTelemetryActivate();
        return {
            getBaseDir: () => exports.baseDir,
            getEnv: () => exports.env,
            onDidChangeEnv: (listener, thisArg) => exports.onDidChangeEnv(listener, thisArg),
        };
    });
}
exports.activate = activate;
function deactivate() {
    subscriptions.forEach(disposable => disposable.dispose());
}
exports.deactivate = deactivate;
/**
 * Activates components which require a ROS env.
 */
function activateEnvironment(context) {
    // Clear existing disposables.
    while (subscriptions.length > 0) {
        subscriptions.pop().dispose();
    }
    if (typeof exports.env.ROS_DISTRO === "undefined") {
        return;
    }
    if (typeof exports.env.ROS_VERSION === "undefined") {
        return;
    }
    // http://www.ros.org/reps/rep-0149.html#environment-variables
    // Learn more about ROS_VERSION definition.
    ros_1.selectROSApi(exports.env.ROS_VERSION);
    ros_1.rosApi.setContext(context, exports.env);
    subscriptions.push(ros_1.rosApi.activateCoreMonitor());
    subscriptions.push(buildtool.BuildTool.registerTaskProvider());
    debug_manager.registerRosDebugManager(context);
    // register plugin commands
    subscriptions.push(vscode.commands.registerCommand(Commands.CreateCatkinPackage, () => {
        buildtool.BuildTool.createPackage(context);
    }), vscode.commands.registerCommand(Commands.CreateTerminal, () => {
        ros_utils.createTerminal(context);
    }), vscode.commands.registerCommand(Commands.GetDebugSettings, () => {
        debug_utils.getDebugSettings(context);
    }), vscode.commands.registerCommand(Commands.ShowCoreStatus, () => {
        ros_1.rosApi.showCoreMonitor();
    }), vscode.commands.registerCommand(Commands.StartRosCore, () => {
        ros_1.rosApi.startCore();
    }), vscode.commands.registerCommand(Commands.TerminateRosCore, () => {
        ros_1.rosApi.stopCore();
    }), vscode.commands.registerCommand(Commands.UpdateCppProperties, () => {
        ros_build_utils.updateCppProperties(context);
    }), vscode.commands.registerCommand(Commands.UpdatePythonPath, () => {
        ros_build_utils.updatePythonPath(context);
    }), vscode.commands.registerCommand(Commands.Rosrun, () => {
        ros_cli.rosrun(context);
    }), vscode.commands.registerCommand(Commands.Roslaunch, () => {
        ros_cli.roslaunch(context);
    }), vscode.commands.registerCommand(Commands.PreviewURDF, () => {
        previewManager_1.default.INSTANCE.preview(vscode.window.activeTextEditor.document.uri);
    }));
    // Generate config files if they don't already exist.
    ros_build_utils.createConfigFiles();
}
/**
 * Loads the ROS environment, and prompts the user to select a distro if required.
 */
function sourceRosAndWorkspace() {
    return __awaiter(this, void 0, void 0, function* () {
        exports.env = undefined;
        const config = vscode_utils.getExtensionConfiguration();
        const distro = config.get("distro", "");
        let setupScriptExt;
        if (process.platform === "win32") {
            setupScriptExt = ".bat";
        }
        else {
            setupScriptExt = ".bash";
        }
        if (distro) {
            try {
                let globalInstallPath;
                if (process.platform === "win32") {
                    globalInstallPath = path.join("C:", "opt", "ros", `${distro}`, "x64");
                }
                else {
                    globalInstallPath = path.join("/", "opt", "ros", `${distro}`);
                }
                let setupScript = path.format({
                    dir: globalInstallPath,
                    name: "setup",
                    ext: setupScriptExt,
                });
                exports.env = yield ros_utils.sourceSetupFile(setupScript, {});
            }
            catch (err) {
                vscode.window.showErrorMessage(`Could not source the setup file for ROS distro "${distro}".`);
            }
        }
        else if (process.env.ROS_DISTRO) {
            exports.env = process.env;
        }
        else {
            const installedDistros = yield ros_utils.getDistros();
            if (!installedDistros.length) {
                throw new Error("No ROS distro found!");
            }
            else if (installedDistros.length === 1) {
                // if there is only one distro installed, directly choose it
                config.update("distro", installedDistros[0]);
            }
            else {
                const message = "The ROS distro is not configured.";
                const configure = "Configure";
                if ((yield vscode.window.showErrorMessage(message, configure)) === configure) {
                    config.update("distro", yield vscode.window.showQuickPick(installedDistros));
                }
            }
        }
        // Source the workspace setup over the top.
        let workspaceDevelPath;
        workspaceDevelPath = path.join(`${exports.baseDir}`, "devel_isolated");
        if (!(yield pfs.exists(workspaceDevelPath))) {
            workspaceDevelPath = path.join(`${exports.baseDir}`, "devel");
        }
        let wsSetupScript = path.format({
            dir: workspaceDevelPath,
            name: "setup",
            ext: setupScriptExt,
        });
        if (exports.env && typeof exports.env.ROS_DISTRO !== "undefined" && (yield pfs.exists(wsSetupScript))) {
            try {
                exports.env = yield ros_utils.sourceSetupFile(wsSetupScript, exports.env);
            }
            catch (_err) {
                vscode.window.showErrorMessage("Failed to source the workspace setup file.");
            }
        }
        // Notify listeners the environment has changed.
        onEnvChanged.fire();
    });
}
//# sourceMappingURL=extension.js.map