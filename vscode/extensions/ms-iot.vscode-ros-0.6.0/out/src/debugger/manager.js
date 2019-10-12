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
const vscode = require("vscode");
const ros_provider = require("./configuration/providers/ros");
const attach_resolver = require("./configuration/resolvers/attach");
const launch_resolver = require("./configuration/resolvers/launch");
class RosDebugManager {
    constructor() {
        this.configProvider = new ros_provider.RosDebugConfigurationProvider();
        this.attachResolver = new attach_resolver.AttachResolver();
        this.launchResolver = new launch_resolver.LaunchResolver();
    }
    provideDebugConfigurations(folder, token) {
        return __awaiter(this, void 0, void 0, function* () {
            return this.configProvider.provideDebugConfigurations(folder, token);
        });
    }
    resolveDebugConfiguration(folder, config, token) {
        return __awaiter(this, void 0, void 0, function* () {
            if (config.request === "attach") {
                return this.attachResolver.resolveDebugConfiguration(folder, config, token);
            }
            else if (config.request === "launch") {
                return this.launchResolver.resolveDebugConfiguration(folder, config, token);
            }
        });
    }
}
function registerRosDebugManager(context) {
    context.subscriptions.push(vscode.debug.registerDebugConfigurationProvider("ros", new RosDebugManager()));
}
exports.registerRosDebugManager = registerRosDebugManager;
//# sourceMappingURL=manager.js.map