#pragma once
#include <Arduino.h>

// Wall-clock time backed by nRF52840 RTC2 (32.768 kHz LFCLK).
//
// RTC2 counts continuously while the MCU is powered and not in System OFF.
// Since powerOff() now uses System ON idle, the RTC keeps ticking through
// "deep sleep"; wall-clock is only lost on a true reset (brownout, firmware
// flash, battery removal). In those cases we restore a "last-known" epoch
// from LittleFS and mark the clock TIME_STALE until the phone re-syncs.
//
// Time is always stored / exchanged as Unix epoch seconds in UTC. The
// phone app is responsible for applying the user's local timezone.

enum DeviceTimeStatus : uint8_t {
  TIME_UNKNOWN = 0,  // Never synced, no persisted value -> getDeviceTime() returns 0
  TIME_STALE   = 1,  // Loaded from flash (valid reference point, but may drift if
                     //                     sleep/off gap was long)
  TIME_FRESH   = 2,  // Synced from BLE this power-cycle
};

// Set up RTC2 (prescaler, overflow IRQ, LFCLK) and load last-known epoch
// from flash. Must be called after initStorage() so LittleFS is mounted.
void initDeviceTime();

// Call periodically from the main loop. Handles periodic flash persistence
// (every 5 minutes while running) so a sudden brownout doesn't lose too
// much wall-clock accuracy.
void maintainDeviceTime();

// Accept a new wall-clock time from the phone (BLE TIME=<epoch>).
// Performs sanity checks, logs any large jump, persists to flash (if the
// new value is sufficiently different from the last saved value).
void setDeviceTime(uint32_t epochSeconds);

// Returns current Unix epoch seconds, or 0 if time has never been known.
uint32_t getDeviceTime();

// Raw monotonic tick counter. Survives the 32-bit millis() wrap. Starts
// at 0 every time initDeviceTime() runs (i.e. every boot). Useful for
// timestamping events that need to survive late-sync backfill.
uint64_t getDeviceTicks();

// Convert a past tick value to an epoch timestamp, assuming the current
// RTC/epoch reference. Returns 0 if time is not yet known.
uint32_t ticksToEpoch(uint64_t ticks);

// True if we have any notion of time (fresh or stale).
bool isDeviceTimeSynced();

DeviceTimeStatus getDeviceTimeStatus();

// Seconds since boot (RTC2-based, monotonic, survives millis() wrap).
uint32_t getDeviceUptimeSeconds();

// Seconds since the last successful BLE TIME= sync this power-cycle.
// Returns UINT32_MAX if never synced this power-cycle (use with the
// TIME_FRESH status to differentiate).
uint32_t getSecondsSinceSync();

// Force a flush of the current epoch to flash. Called before deep sleep
// and from maintainDeviceTime() on the periodic schedule. Deduplicates
// writes to protect flash endurance.
void persistDeviceTime();

// Format epoch as "YYYY-MM-DD HH:MM:SS" in UTC. Writes at most outLen-1
// chars. Writes "----" for epoch==0.
void formatEpochUTC(uint32_t epochSeconds, char *out, size_t outLen);

// Format epoch as ISO-8601 "YYYY-MM-DDTHH:MM:SSZ" in UTC. Empty string
// if epoch==0.
void formatEpochISO(uint32_t epochSeconds, char *out, size_t outLen);

// Optional timezone offset in seconds (e.g. +05:30 IST = 19800).
// Used by the *local* time formatters for nicer serial logs. Defaults
// to 0 (UTC). Not persisted — the phone re-pushes TZ on each connect.
void setDeviceTZOffset(int32_t offsetSeconds);
int32_t getDeviceTZOffset();

// Format epoch into the user's local time as "YYYY-MM-DD HH:MM:SS ±HH:MM".
// Writes "----" for epoch==0.
void formatEpochLocal(uint32_t epochSeconds, char *out, size_t outLen);
