# DroneCAN firmware update over a MAVLink bridge

Flashes a CORVON G-series node over the CAN bus, through an ArduPilot flight
controller's MAV_CMD_CAN_FORWARD bridge. No ST-Link, no USB-TTL - the 4-pin
CAN cable the unit ships with is the whole physical interface.

    python3 canup.py <release .apj>

Verified on hardware 2026-08-27: G1, ArduPilot FC on /dev/cu.usbmodem*,
164KB in ~11s, parameters preserved. Requires `pip install dronecan pymavlink`
and CAN_P1_DRIVER=1 on the FC. Edit FCDEV at the top for the FC's port.

Three things that will bite anyone touching this script:

1. The file served to the bootloader is the raw image decoded out of the .apj
   (base64+zlib), NOT the .apj itself. The script does this extraction.
2. dronecan's mavcan driver spawns a child process; on macOS (spawn start
   method) the calling script must guard its body with `if __name__ ==
   "__main__"` or the child re-imports it and dies at once.
3. `node.spin()` raises TransferError when the MAVLink forwarding drops a
   frame mid-transfer, which it does routinely. An unguarded spin kills the
   file server while the bootloader is mid-download. Harmless - the old app
   stays valid until the image actually lands, and the bootloader waits - but
   the update stalls until Begin is sent again. The spin() wrapper here eats
   those.

The interruption case was hit live: the server died at the Begin stage, the
node sat in its bootloader (node info vcs 0x00000000) waiting for a file, and
a plain rerun of the script picked it up and finished.
