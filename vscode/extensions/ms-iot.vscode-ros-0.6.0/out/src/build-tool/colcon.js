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
var __asyncValues = (this && this.__asyncValues) || function (o) {
    if (!Symbol.asyncIterator) throw new TypeError("Symbol.asyncIterator is not defined.");
    var m = o[Symbol.asyncIterator], i;
    return m ? m.call(o) : (o = typeof __values === "function" ? __values(o) : o[Symbol.iterator](), i = {}, verb("next"), verb("throw"), verb("return"), i[Symbol.asyncIterator] = function () { return this; }, i);
    function verb(n) { i[n] = o[n] && function (v) { return new Promise(function (resolve, reject) { v = o[n](v), settle(resolve, reject, v.done, v.value); }); }; }
    function settle(resolve, reject, d, v) { Promise.resolve(v).then(function(v) { resolve({ value: v, done: d }); }, reject); }
};
Object.defineProperty(exports, "__esModule", { value: true });
const child_process = require("child_process");
const vscode = require("vscode");
const extension = require("../extension");
/**
 * Provides colcon build and test tasks.
 */
class ColconProvider {
    provideTasks(token) {
        const buildCommand = "colcon build";
        const testCommand = "colcon test";
        const build = new vscode.Task({ type: "colcon" }, vscode.TaskScope.Workspace, "build", "colcon", new vscode.ShellExecution(buildCommand, {
            env: extension.env,
        }), []);
        build.group = vscode.TaskGroup.Build;
        const test = new vscode.Task({ type: "colcon" }, vscode.TaskScope.Workspace, "test", "colcon", new vscode.ShellExecution(testCommand, {
            env: extension.env,
        }), []);
        test.group = vscode.TaskGroup.Test;
        return [build, test];
    }
    resolveTask(task, token) {
        return undefined;
    }
}
exports.ColconProvider = ColconProvider;
function isApplicable(dir) {
    var e_1, _a;
    return __awaiter(this, void 0, void 0, function* () {
        const opts = { dir, env: extension.env };
        const { stdout, stderr } = yield child_process.exec("colcon -h", opts);
        try {
            for (var stderr_1 = __asyncValues(stderr), stderr_1_1; stderr_1_1 = yield stderr_1.next(), !stderr_1_1.done;) {
                const line = stderr_1_1.value;
                return false;
            }
        }
        catch (e_1_1) { e_1 = { error: e_1_1 }; }
        finally {
            try {
                if (stderr_1_1 && !stderr_1_1.done && (_a = stderr_1.return)) yield _a.call(stderr_1);
            }
            finally { if (e_1) throw e_1.error; }
        }
        return true;
    });
}
exports.isApplicable = isApplicable;
//# sourceMappingURL=colcon.js.map