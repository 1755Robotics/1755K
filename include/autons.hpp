/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#pragma once
#include "main.h"

// Enum for autonomous routines
enum class AutonRoutine {
    LeftQual,
    RightQual,
    Skills,
    LinPID,
    AngPID,
    None,
};

// Global variable to store the selected auton
extern AutonRoutine selectedAuton;

// Functions
void autonSelectorTask(void*);
void startAutonSelector();      // Select auton during pre-auton
void runSelectedAuton();   // Run the selected auton

// Autons
void leftQual();
void rightQual();
void leftElim();
void rightElim();
void autonSkills();
void lin_pid_tuning_test();
void ang_pid_tuning_test();

