"use strict";
// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.
Object.defineProperty(exports, "__esModule", { value: true });
const vscode = require("vscode");
const path = require("path");
function getPackageInfo(context) {
    const metadataFile = "package.json";
    const metadata = require(path.join(context.extensionPath, metadataFile));
    if (metadata && ("name" in metadata) && ("version" in metadata) && ("aiKey" in metadata)) {
        return {
            name: metadata.name,
            version: metadata.version,
            aiKey: metadata.aiKey,
        };
    }
    return undefined;
}
exports.getPackageInfo = getPackageInfo;
function getExtensionConfiguration() {
    const rosConfigurationName = "ros";
    return vscode.workspace.getConfiguration(rosConfigurationName);
}
exports.getExtensionConfiguration = getExtensionConfiguration;
function createOutputChannel() {
    return vscode.window.createOutputChannel("ROS");
}
exports.createOutputChannel = createOutputChannel;
//# sourceMappingURL=vscode-utils.js.map