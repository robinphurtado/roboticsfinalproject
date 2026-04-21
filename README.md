# CS556 Final Project 

## Project Overview
- Isabella Lawlor & Robin Hurtado
- CS 556 Spring 2026
- Brief description of the goal: 

## Hardware
- Pololu 3pi+ 32U4
- Sonar sensors, IR line sensors, OLED
- servo, buzzer

## File Structure
- finalproject.ino — main state machine and loop
- PDcontroller.h/.cpp — PD control for wall following and line following
- odometry.h/.cpp — pose tracking (x, y, θ)
- sonar.h/.cpp — distance sensing
- printOLED.h/.cpp — OLED display helper

## Phase Descriptions
### Phase 1: Navigation & Localization
- Navigation method used: wall following
- How cells are tracked: visitedCells array
- How path is recorded: movementLog array

### Phase 2: Return to Dock
- How return path is computed: reverse movementLog
- Dock detection method:
- Completion signal: buzzer

### Phase 3: Pick Detection & Service
- Black square detection: center IR sensor > BLACK_THRESHOLD
- serviceBin() behavior: stop, spin, OLED update

### Phase 4: Speed Management

## Code Design
### Modularization

### Parameter Settings

### Commenting

## How to Run

## Known Limitations
