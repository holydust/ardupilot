#!/usr/bin/env python3
# Flash a CORVON G-series node over DroneCAN, through the ArduPilot FC's
# MAV_CMD_CAN_FORWARD bridge. Serves the image with the dronecan FileServer
# and asks the node to BeginFirmwareUpdate; the AP bootloader pulls the file.
import sys, os, time, json, base64, zlib
import dronecan

def main():
    def spin(node, t):
        # MAVLink CAN forwarding drops frames now and then; a broken
        # multi-frame transfer must not kill the file server
        end = time.time() + t
        while time.time() < end:
            try:
                node.spin(max(0.05, end - time.time()))
                return
            except dronecan.transport.TransferError:
                pass
            except Exception as e:
                print("spin: %s" % e, flush=True)

    APJ = sys.argv[1]
    FCDEV = "mavcan:/dev/cu.usbmodem31201"
    SP = os.path.dirname(os.path.abspath(__file__))

    apj = json.load(open(APJ))
    img = zlib.decompress(base64.b64decode(apj["image"]))
    binpath = os.path.join(SP, "fw.bin")
    open(binpath, "wb").write(img)
    print("image: %d bytes, board_id %s, git %s" %
          (len(img), apj.get("board_id"), apj.get("git_identity")), flush=True)

    node = dronecan.make_node(FCDEV, node_id=126, bitrate=1000000)
    monitor = dronecan.app.node_monitor.NodeMonitor(node)

    def find_target():
        for e in monitor.find_all(lambda e: True):
            info = e.info
            if info is not None and b"CORVON-G1" in bytes(info.name):
                return e.node_id, info
        return None, None

    print("discovering nodes...", flush=True)
    deadline = time.time() + 30
    target = None
    while time.time() < deadline and target is None:
        spin(node, 0.5)
        target, info = find_target()
    if target is None:
        print("FAIL: CORVON-G1 not seen on the bus"); sys.exit(1)
    swv = info.software_version
    print("target: node %d, sw vcs 0x%08x" % (target, swv.vcs_commit), flush=True)

    fs = dronecan.app.file_server.FileServer(node, lookup_paths=[SP])

    def on_begin(e):
        print("BeginFirmwareUpdate response:", e.response if e else "(timeout)", flush=True)

    req = dronecan.uavcan.protocol.file.BeginFirmwareUpdate.Request(
        source_node_id=node.node_id,
        image_file_remote_path=dronecan.uavcan.protocol.file.Path(path="fw.bin"))
    node.request(req, target, on_begin)

    print("waiting for the node to pull the image over CAN...", flush=True)
    t0 = time.time(); last = 0
    want = int(apj["git_identity"], 16)
    while time.time() - t0 < 420:
        spin(node, 0.5)
        if time.time() - last > 10:
            last = time.time()
            tid, tinfo = find_target()
            if tinfo is not None:
                vcs = tinfo.software_version.vcs_commit
                print("  %3.0fs node %s sw 0x%08x" % (time.time()-t0, tid, vcs), flush=True)
                if vcs == want:
                    print("SUCCESS: node reports 0x%08x" % vcs); sys.exit(0)
    print("TIMEOUT: node never reported the new version"); sys.exit(2)

if __name__ == "__main__":
    main()
