# SPDX-License-Identifier: Apache-2.0
# Source from OpenOCD after selecting an nRF54L15 target.
# Commit the final partial write buffer before reset or verification.

proc rram_wait_ready {} {
    for {set attempts 0} {$attempts < 100} {incr attempts} {
        if {[lindex [read_memory 0x5004b400 32 1] 0] == 1} {
            return
        }
        sleep 1
    }
    error "nRF54L15 RRAMC READY timeout"
}

proc rram_commit {} {
    mww 0x5004b008 1
    rram_wait_ready
    mww 0x5004b500 0
}
