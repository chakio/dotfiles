"use strict";
// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.
Object.defineProperty(exports, "__esModule", { value: true });
const vscode_extension_telemetry_1 = require("vscode-extension-telemetry");
const vscode_utils = require("./vscode-utils");
let reporterSingleton;
function getTelemetryReporter(context) {
    if (reporterSingleton) {
        return reporterSingleton;
    }
    const packageInfo = vscode_utils.getPackageInfo(context);
    if (packageInfo) {
        reporterSingleton = new vscode_extension_telemetry_1.default(packageInfo.name, packageInfo.version, packageInfo.aiKey);
        context.subscriptions.push(reporterSingleton);
    }
    return reporterSingleton;
}
var TelemetryEvent;
(function (TelemetryEvent) {
    TelemetryEvent["activate"] = "activate";
    TelemetryEvent["command"] = "command";
})(TelemetryEvent || (TelemetryEvent = {}));
class SimpleReporter {
    constructor(context) {
        this.telemetryReporter = getTelemetryReporter(context);
    }
    sendTelemetryActivate() {
        if (!this.telemetryReporter) {
            return;
        }
        this.telemetryReporter.sendTelemetryEvent(TelemetryEvent.activate);
    }
    sendTelemetryCommand(commandName) {
        if (!this.telemetryReporter) {
            return;
        }
        this.telemetryReporter.sendTelemetryEvent(TelemetryEvent.command, {
            name: commandName,
        });
    }
}
function getReporter(context) {
    return (new SimpleReporter(context));
}
exports.getReporter = getReporter;
//# sourceMappingURL=telemetry-helper.js.map