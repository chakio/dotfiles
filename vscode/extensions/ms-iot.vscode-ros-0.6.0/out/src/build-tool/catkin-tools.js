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
const extension = require("../extension");
const common = require("./common");
/**
 * Provides catkin tools build and test tasks.
 */
class CatkinToolsProvider {
    provideTasks(token) {
        let buildCommand;
        let testCommand;
        buildCommand = `catkin build --workspace "${extension.baseDir}"`;
        testCommand = `${buildCommand} --catkin-make-args run_tests`;
        const make = new vscode.Task({ type: "catkin" }, "make", "catkin");
        make.execution = new vscode.ShellExecution(buildCommand, {
            env: extension.env,
        });
        make.group = vscode.TaskGroup.Build;
        make.problemMatchers = ["$catkin-gcc"];
        const test = new vscode.Task({ type: "catkin", target: "run_tests" }, "run_tests", "catkin");
        test.execution = new vscode.ShellExecution(testCommand, {
            env: extension.env,
        });
        test.group = vscode.TaskGroup.Test;
        return [make, test];
    }
    resolveTask(task, token) {
        return undefined;
    }
}
exports.CatkinToolsProvider = CatkinToolsProvider;
/**
 * Interacts with the user to run a `catkin create pkg` command.
 */
function createPackage(uri) {
    return __awaiter(this, void 0, void 0, function* () {
        const createPkgCommand = (dependencies, name) => {
            return `catkin create pkg --catkin-deps ${dependencies} -- ${name}`;
        };
        return common._createPackage(createPkgCommand);
    });
}
exports.createPackage = createPackage;
//# sourceMappingURL=catkin-tools.js.map