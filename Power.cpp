/*
 * Copyright (c) 2019, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of The Linux Foundation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#define LOG_TAG "android.hardware.power@1.2-service-qti"

#include "Power.h"
#include <log/log.h>
#include "power-common.h"

/* RPM runs at 19.2Mhz. Divide by 19200 for msec */
#define RPM_CLK 19200

extern struct stat_pair rpm_stat_map[];

namespace android {
namespace hardware {
namespace power {
namespace V1_2 {
namespace implementation {

using ::android::hardware::hidl_vec;
using ::android::hardware::Return;
using ::android::hardware::Void;
using ::android::hardware::power::V1_0::Feature;
using ::android::hardware::power::V1_0::PowerHint;
using ::android::hardware::power::V1_0::PowerStatePlatformSleepState;
using ::android::hardware::power::V1_0::Status;
using ::android::hardware::power::V1_1::PowerStateSubsystem;

Power::Power() {
    power_init();
}

Return<void> Power::setInteractive(bool interactive) {
    set_interactive(interactive ? 1 : 0);
    return Void();
}

Return<void> Power::powerHint(PowerHint_1_0 hint, int32_t data) {
    power_hint(static_cast<power_hint_t>(hint), data ? (&data) : NULL);
    return Void();
}

Return<void> Power::setFeature(Feature feature, bool activate) {
    set_feature(static_cast<feature_t>(feature), activate ? 1 : 0);
    return Void();
}

Return<void> Power::getPlatformLowPowerStats(getPlatformLowPowerStats_cb _hidl_cb) {
    hidl_vec<PowerStatePlatformSleepState> states;
    uint64_t stats[MAX_PLATFORM_STATS * MAX_RPM_PARAMS] = {0};
#ifndef LEGACY_STATS
    uint64_t *values;
#endif
    struct PowerStatePlatformSleepState *state;
    int ret;

    ret = extract_platform_stats(stats);
    if (ret != 0) {
        states.resize(0);
        goto done;
    }

#ifndef LEGACY_STATS
    states.resize(PLATFORM_SLEEP_MODES_COUNT);

    /* Update statistics for XO_shutdown */
    state = &states[RPM_MODE_XO];
    state->name = "XO_shutdown";
    values = stats + (RPM_MODE_XO * MAX_RPM_PARAMS);

    state->residencyInMsecSinceBoot = values[1];
    state->totalTransitions = values[0];
    state->supportedOnlyInSuspend = false;
    state->voters.resize(XO_VOTERS);
    for(size_t i = 0; i < XO_VOTERS; i++) {
        int voter = i + XO_VOTERS_START;
        state->voters[i].name = rpm_stat_map[voter].label;
        values = stats + (voter * MAX_RPM_PARAMS);
        state->voters[i].totalTimeInMsecVotedForSinceBoot = values[0] / RPM_CLK;
        state->voters[i].totalNumberOfTimesVotedSinceBoot = values[1];
    }

    /* Update statistics for VMIN state */
    state = &states[RPM_MODE_VMIN];
    state->name = "VMIN";
    values = stats + (RPM_MODE_VMIN * MAX_RPM_PARAMS);

    state->residencyInMsecSinceBoot = values[1];
    state->totalTransitions = values[0];
    state->supportedOnlyInSuspend = false;
    state->voters.resize(VMIN_VOTERS);
#else
    states.resize(RPM_MODE_MAX);

    /* Update statistics for XO_shutdown */
    state = &states[RPM_MODE_XO];
    state->name = "XO_shutdown";

    state->residencyInMsecSinceBoot = stats[ACCUMULATED_VLOW_TIME];
    state->totalTransitions = stats[VLOW_COUNT];
    state->supportedOnlyInSuspend = false;
    state->voters.resize(XO_VOTERS);

    /* Update statistics for APSS voter */
    state->voters[0].name = "APSS";
    state->voters[0].totalTimeInMsecVotedForSinceBoot =
        stats[XO_ACCUMULATED_DURATION_APSS] / RPM_CLK;
    state->voters[0].totalNumberOfTimesVotedSinceBoot = stats[XO_COUNT_APSS];

    /* Update statistics for MPSS voter */
    state->voters[1].name = "MPSS";
    state->voters[1].totalTimeInMsecVotedForSinceBoot =
        stats[XO_ACCUMULATED_DURATION_MPSS] / RPM_CLK;
    state->voters[1].totalNumberOfTimesVotedSinceBoot = stats[XO_COUNT_MPSS];

    /* Update statistics for ADSP voter */
    state->voters[2].name = "ADSP";
    state->voters[2].totalTimeInMsecVotedForSinceBoot =
        stats[XO_ACCUMULATED_DURATION_ADSP] / RPM_CLK;
    state->voters[2].totalNumberOfTimesVotedSinceBoot = stats[XO_COUNT_ADSP];

    /* Update statistics for SLPI voter */
    state->voters[3].name = "SLPI";
    state->voters[3].totalTimeInMsecVotedForSinceBoot =
        stats[XO_ACCUMULATED_DURATION_SLPI] / RPM_CLK;
    state->voters[3].totalNumberOfTimesVotedSinceBoot = stats[XO_COUNT_SLPI];

    /* Update statistics for VMIN state */
    state = &states[RPM_MODE_VMIN];

    state->name = "VMIN";
    state->residencyInMsecSinceBoot = stats[ACCUMULATED_VMIN_TIME];
    state->totalTransitions = stats[VMIN_COUNT];
    state->supportedOnlyInSuspend = false;
    state->voters.resize(VMIN_VOTERS);
#endif

done:
    _hidl_cb(states, Status::SUCCESS);
    return Void();
}

#ifndef NO_WLAN_STATS
static int get_wlan_low_power_stats(struct PowerStateSubsystem &subsystem) {

    uint64_t stats[WLAN_POWER_PARAMS_COUNT] = {0};
    struct PowerStateSubsystemSleepState *state;
    int ret;

    ret = extract_wlan_stats(stats);
    if (ret)
        return ret;

    subsystem.name = "wlan";
    subsystem.states.resize(WLAN_STATES_COUNT);

    /* Update statistics for Active State */
    state = &subsystem.states[WLAN_STATE_ACTIVE];
    state->name = "Active";
    state->residencyInMsecSinceBoot = stats[CUMULATIVE_TOTAL_ON_TIME_MS];
    state->totalTransitions = stats[DEEP_SLEEP_ENTER_COUNTER];
    state->lastEntryTimestampMs = 0; //FIXME need a new value from Qcom
    state->supportedOnlyInSuspend = false;

    /* Update statistics for Deep-Sleep state */
    state = &subsystem.states[WLAN_STATE_DEEP_SLEEP];
    state->name = "Deep-Sleep";
    state->residencyInMsecSinceBoot = stats[CUMULATIVE_SLEEP_TIME_MS];
    state->totalTransitions = stats[DEEP_SLEEP_ENTER_COUNTER];
    state->lastEntryTimestampMs = stats[LAST_DEEP_SLEEP_ENTER_TSTAMP_MS];
    state->supportedOnlyInSuspend = false;

    return 0;
}
#endif

Return<void> Power::getSubsystemLowPowerStats(getSubsystemLowPowerStats_cb _hidl_cb) {
    hidl_vec<PowerStateSubsystem> subsystems;
#ifndef NO_WLAN_STATS
    int ret;

    subsystems.resize(subsystem_type::SUBSYSTEM_COUNT);

    //We currently have only one Subsystem for WLAN
    ret = get_wlan_low_power_stats(subsystems[subsystem_type::SUBSYSTEM_WLAN]);
    if (ret != 0)
        goto done;

done:
#else
    subsystems.resize(0);
#endif
    _hidl_cb(subsystems, Status::SUCCESS);
    return Void();
}

Return<void> Power::powerHintAsync(PowerHint_1_0 hint, int32_t data) {
    return powerHint(hint, data);
}

Return<void> Power::powerHintAsync_1_2(PowerHint_1_2 hint, int32_t data) {
    return powerHint(static_cast<PowerHint_1_0>(hint), data);
}

}  // namespace implementation
}  // namespace V1_2
}  // namespace power
}  // namespace hardware
}  // namespace android
