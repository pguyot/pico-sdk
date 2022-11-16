/*
 * Copyright (c) 2022-2025 Paul Guyot <pguyot@kallisys.net>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pico/cond.h"

void cond_init(cond_t *cond) {
    lock_init(&cond->core, next_striped_spin_lock_num());
    cond->waiter = LOCK_INVALID_OWNER_ID;
    cond->broadcast_count = 0;
    cond->signaled = false;
    __mem_fence_release();
}

bool __time_critical_func(cond_wait_until)(cond_t *cond, mutex_t *mtx, absolute_time_t until) {
    bool success = true;
    lock_owner_id_t caller = lock_get_caller_owner_id();

    // Try to acquire the condition variable, i.e. be the first waiter.
    // Acquire the condition variable lock first.
    uint32_t save1 = spin_lock_blocking(cond->core.spin_lock);
    uint64_t current_broadcast = cond->broadcast_count;
    if (lock_is_owner_id_valid(cond->waiter)) {
        // There is a valid owner of the condition variable: we are not the
        // first waiter.
        lock_internal_spin_unlock_with_notify(&cond->core, save1);

        // Release the mutex
        mutex_exit(mtx);

        do {
            uint32_t save2 = spin_lock_blocking(cond->core.spin_lock);
            if (cond->broadcast_count != current_broadcast) {
                // Condition variable was broadcast while we were waiting to
                // own it.
                lock_internal_spin_unlock_with_notify(&cond->core, save2);
                break;
            }
            if (!lock_is_owner_id_valid(cond->waiter)) {
                cond->waiter = caller;
                lock_internal_spin_unlock_with_notify(&cond->core, save2);
                break;
            }
            if (is_at_the_end_of_time(until)) {
                lock_internal_spin_unlock_with_wait(&cond->core, save2);
            } else if (lock_internal_spin_unlock_with_best_effort_wait_or_timeout(&cond->core, save2, until)) {
                // timed out
                success = false;
                break;
            }
        } while (true);
    } else {
        cond->waiter = caller;
        lock_internal_spin_unlock_with_notify(&cond->core, save1);

        // Release the mutex
        mutex_exit(mtx);
    }

    if (success) {
        // Wait for the signal
        do {
            uint32_t save3 = spin_lock_blocking(cond->core.spin_lock);
            if (cond->signaled) {
                cond->waiter = LOCK_INVALID_OWNER_ID;
                cond->signaled = false;
                lock_internal_spin_unlock_with_notify(&cond->core, save3);
                break;
            }
            if (!success) {
                cond->waiter = LOCK_INVALID_OWNER_ID;
                lock_internal_spin_unlock_with_notify(&cond->core, save3);
                break;
            }
            if (is_at_the_end_of_time(until)) {
                lock_internal_spin_unlock_with_wait(&cond->core, save3);
            } else if (lock_internal_spin_unlock_with_best_effort_wait_or_timeout(&cond->core, save3, until)) {
                // timed out
                success = false;
            }
        } while (true);
    }

    // Acquire the mutex
    mutex_enter_blocking(mtx);

    return success;
}

bool __time_critical_func(cond_wait_timeout_ms)(cond_t *cond, mutex_t *mtx, uint32_t timeout_ms) {
    return cond_wait_until(cond, mtx, make_timeout_time_ms(timeout_ms));
}

bool __time_critical_func(cond_wait_timeout_us)(cond_t *cond, mutex_t *mtx, uint32_t timeout_us) {
    return cond_wait_until(cond, mtx, make_timeout_time_us(timeout_us));
}

void __time_critical_func(cond_wait)(cond_t *cond, mutex_t *mtx) {
    cond_wait_until(cond, mtx, at_the_end_of_time);
}

void __time_critical_func(cond_signal)(cond_t *cond) {
    uint32_t save = spin_lock_blocking(cond->core.spin_lock);
    if (lock_is_owner_id_valid(cond->waiter)) {
        // We have a waiter, we can signal.
        cond->signaled = true;
        lock_internal_spin_unlock_with_notify(&cond->core, save);
    } else {
        spin_unlock(cond->core.spin_lock, save);
    }
}

void __time_critical_func(cond_broadcast)(cond_t *cond) {
    uint32_t save = spin_lock_blocking(cond->core.spin_lock);
    if (lock_is_owner_id_valid(cond->waiter)) {
        // We have a waiter, we can broadcast.
        cond->signaled = true;
        cond->broadcast_count++;
        lock_internal_spin_unlock_with_notify(&cond->core, save);
    } else {
        spin_unlock(cond->core.spin_lock, save);
    }
}
