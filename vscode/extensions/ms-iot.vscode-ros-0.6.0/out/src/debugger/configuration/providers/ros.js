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
// interact with the user to create a roslaunch or rosrun configuration
class RosDebugConfigurationProvider {
    provideDebugConfigurations(folder, token) {
        return __awaiter(this, void 0, void 0, function* () {
            const configs = undefined;
            // this could be implemented to provide debug configurations interactively
            // generate configurations with snippets defined in package.json for now
            return configs;
        });
    }
}
exports.RosDebugConfigurationProvider = RosDebugConfigurationProvider;
//# sourceMappingURL=ros.js.map