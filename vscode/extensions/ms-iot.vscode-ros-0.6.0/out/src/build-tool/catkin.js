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
 * Provides catkin build and test tasks
 * including catkin_make and catkin_make_isolated
 */
class CatkinProvider {
    provideTasks(token) {
        const tasksCatkinMake = this.provideCatkinMakeTasks();
        const tasksCatkinMakeIsolated = this.provideCatkinMakeIsolatedTasks();
        return [...tasksCatkinMake, ...tasksCatkinMakeIsolated];
    }
    resolveTask(task, token) {
        return undefined;
    }
    provideCatkinMakeTasks() {
        const catkinMakeDefinition = {
            type: "catkin_make",
            target: "build",
        };
        const make = new vscode.Task(catkinMakeDefinition, "build", "catkin_make");
        const buildCommand = `catkin_make --directory "${extension.baseDir}"`;
        make.execution = new vscode.ShellExecution(buildCommand, {
            env: extension.env,
        });
        make.group = vscode.TaskGroup.Build;
        make.problemMatchers = ["$catkin-gcc"];
        const catkinMakeRunTestsDefinition = {
            type: "catkin_make",
            target: "run_tests",
        };
        const test = new vscode.Task(catkinMakeRunTestsDefinition, "run_tests", "catkin_make");
        const testCommand = `${buildCommand} run_tests`;
        test.execution = new vscode.ShellExecution(testCommand, {
            env: extension.env,
        });
        test.group = vscode.TaskGroup.Test;
        return [make, test];
    }
    provideCatkinMakeIsolatedTasks() {
        const catkinMakeIsolatedDefinition = {
            type: "catkin_make_isolated",
            target: "build",
        };
        const make = new vscode.Task(catkinMakeIsolatedDefinition, "build", "catkin_make_isolated");
        const buildCommand = `catkin_make_isolated --directory "${extension.baseDir}"`;
        make.execution = new vscode.ShellExecution(buildCommand, {
            env: extension.env,
        });
        make.group = vscode.TaskGroup.Build;
        make.problemMatchers = ["$catkin-gcc"];
        const catkinMakeIsolatedRunTestsDefinition = {
            type: "catkin_make_isolated",
            target: "run_tests",
        };
        const test = new vscode.Task(catkinMakeIsolatedRunTestsDefinition, "run_tests", "catkin_make_isolated");
        const testCommand = `${buildCommand} --catkin-make-args run_tests`;
        test.execution = new vscode.ShellExecution(testCommand, {
            env: extension.env,
        });
        test.group = vscode.TaskGroup.Test;
        return [make, test];
    }
}
exports.CatkinProvider = CatkinProvider;
/**
 * Interacts with the user to run a `catkin_create_pkg` command.
 */
function createPackage(uri) {
    return __awaiter(this, void 0, void 0, function* () {
        const createPkgCommand = (dependencies, name) => {
            return `catkin_create_pkg ${name} ${dependencies}`;
        };
        return common._createPackage(createPkgCommand);
    });
}
exports.createPackage = createPackage;
//# sourceMappingURL=catkin.js.map