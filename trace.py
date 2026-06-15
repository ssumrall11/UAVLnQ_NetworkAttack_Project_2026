"""
trace_mission_path.py
---------------------
Traces MISSION_ITEM (MAVLink) packets across ns-3 node PCAPs to show
which nodes the packet passed through (leader -> relay1 -> relay2 -> follower).

Usage:
    python3 trace_mission_path.py --dir /path/to/pcap/dir
    python3 trace_mission_path.py --files node0.pcap node1.pcap node2.pcap node3.pcap

The script expects one PCAP per ns-3 node, named with a node index
in the filename (e.g. *-0-0.pcap, *-1-0.pcap, ...) or passed in order.

Node roles (4-node multihop topology):
    Node 0  -> Leader      (10.1.1.1 / 00:00:00:00:00:01)
    Node 1  -> Relay 1     (10.1.1.2 / 00:00:00:00:00:02)
    Node 2  -> Relay 2     (10.1.1.3 / 00:00:00:00:00:03)
    Node 3  -> Follower    (10.1.1.4 / 00:00:00:00:00:04)
"""

import argparse
import struct
import os
import glob
import sys
import collections

try:
    from scapy.all import rdpcap, UDP, IP, Dot11
except ImportError:
    print("[ERROR] scapy not installed. Run: pip install scapy --break-system-packages")
    sys.exit(1)


# ── Config ────────────────────────────────────────────────────────────────────

NODE_ROLES = {
    0: "Leader",
    1: "Relay 1",
    2: "Relay 2",
    3: "Follower",
}

NODE_IPS = {
    0: "10.1.1.1",
    1: "10.1.1.2",
    2: "10.1.1.3",
    3: "10.1.1.4",
}

NODE_MACS = {
    0: "00:00:00:00:00:01",
    1: "00:00:00:00:00:02",
    2: "00:00:00:00:00:03",
    3: "00:00:00:00:00:04",
}

MAC_TO_NODE = {v: k for k, v in NODE_MACS.items()}
IP_TO_NODE  = {v: k for k, v in NODE_IPS.items()}

MAVLINK_PORT = 20000


# ── MAVLink decode ────────────────────────────────────────────────────────────

def decode_mission_item(raw_payload):
    """
    Decode a MAVLink v2 MISSION_ITEM frame from raw UDP payload bytes.
    Returns a dict or None if not a valid MISSION_ITEM.
    """
    if len(raw_payload) < 10:
        return None
    if raw_payload[0] != 0xFD:          # MAVLink v2 magic
        return None

    plen   = raw_payload[1]
    seq    = raw_payload[4]
    sysid  = raw_payload[5]
    compid = raw_payload[6]
    msgid  = raw_payload[7] | (raw_payload[8] << 8) | (raw_payload[9] << 16)

    if msgid != 39:                     # MAVLINK_MSG_ID_MISSION_ITEM = 39
        return None

    payload = raw_payload[10:10 + plen]
    if len(payload) < 37:
        return None

    # Wire layout (sorted by field size, MAVLink v2):
    # float32 x4 params, float32 x, float32 y, float32 z  (offset 0-27)
    # uint16  seq, uint16 command                           (offset 28-31)
    # uint8   target_system, target_component, frame,
    #         current, autocontinue                         (offset 32-36)
    p1, p2, p3, p4, x, y, z = struct.unpack_from('<fffffff', payload, 0)
    seq_mi, command          = struct.unpack_from('<HH',      payload, 28)
    target_sys               = payload[32]

    return {
        "mavlink_seq":    seq,
        "sysid":          sysid,
        "target_system":  target_sys,
        "mission_seq":    seq_mi,
        "command":        command,
        "x": x, "y": y, "z": z,
    }


# ── Load PCAPs ────────────────────────────────────────────────────────────────

def load_pcaps(pcap_files):
    """
    Returns dict: node_index -> list of (timestamp, src_ip, dst_ip,
                                          wlan_transmitter_mac, wlan_nexthop_mac,
                                          mission_item_dict)
    """
    node_events = {}

    for node_idx, path in pcap_files.items():
        events = []
        try:
            pkts = rdpcap(path)
        except Exception as e:
            print(f"[WARN] Could not read {path}: {e}")
            continue

        for p in pkts:
            if not (IP in p and UDP in p):
                continue
            if p[UDP].dport != MAVLINK_PORT:
                continue

            raw = bytes(p[UDP].payload)
            info = decode_mission_item(raw)
            if info is None:
                continue

            t        = float(p.time)
            src_ip   = p[IP].src
            dst_ip   = p[IP].dst

            # 802.11 ad-hoc fields:
            #   addr2 = transmitter (who put this frame on air at this hop)
            #   addr1 = receiver MAC resolved by AODV for this hop
            tx_mac   = None
            rx_mac   = None
            if Dot11 in p:
                tx_mac = p[Dot11].addr2
                rx_mac = p[Dot11].addr1

            events.append((t, src_ip, dst_ip, tx_mac, rx_mac, info))

        node_events[node_idx] = sorted(events, key=lambda e: e[0])

    return node_events


# ── Build travel path ─────────────────────────────────────────────────────────

def build_travel_path(node_events, time_tolerance=0.01):
    """
    Group events by approximate send time to reconstruct per-waypoint paths.
    Returns list of waypoint dicts, each with a 'path' list of hop dicts.
    """
    # Collect all unique send times from node 0 (leader sends the packet)
    leader_times = []
    if 0 in node_events:
        for (t, src, dst, tx_mac, rx_mac, info) in node_events[0]:
            leader_times.append(t)

    if not leader_times:
        # Fallback: use any node that saw the packet
        all_times = []
        for node_idx, events in node_events.items():
            for (t, *_) in events:
                all_times.append(t)
        # Cluster by time_tolerance
        all_times.sort()
        leader_times = []
        for t in all_times:
            if not leader_times or t - leader_times[-1] > time_tolerance:
                leader_times.append(t)

    waypoints = []

    for send_time in leader_times:
        wp = {
            "send_time": send_time,
            "path": [],
            "delivered": False,
            "mission_info": None,
        }

        for node_idx in sorted(node_events.keys()):
            for (t, src_ip, dst_ip, tx_mac, rx_mac, info) in node_events[node_idx]:
                if abs(t - send_time) > time_tolerance:
                    continue

                # Identify who transmitted this frame at this hop
                # In ns-3 ad-hoc: addr2 is the transmitter at this hop
                tx_node = MAC_TO_NODE.get(tx_mac) if tx_mac else None
                rx_node = MAC_TO_NODE.get(rx_mac) if rx_mac else None

                hop = {
                    "seen_by_node":  node_idx,
                    "seen_by_role":  NODE_ROLES.get(node_idx, f"Node{node_idx}"),
                    "timestamp":     t,
                    "tx_mac":        tx_mac,
                    "rx_mac":        rx_mac,
                    "tx_node":       tx_node,
                    "rx_node":       rx_node,
                    "src_ip":        src_ip,
                    "dst_ip":        dst_ip,
                }

                wp["path"].append(hop)
                if wp["mission_info"] is None:
                    wp["mission_info"] = info
                if node_idx == 3:
                    wp["delivered"] = True
                break  # one event per node per waypoint

        waypoints.append(wp)

    return waypoints


# ── Print report ──────────────────────────────────────────────────────────────

def print_report(waypoints):
    SEP = "=" * 60

    print(f"\n{SEP}")
    print("  MISSION_ITEM TRAVEL PATH REPORT")
    print(SEP)

    for i, wp in enumerate(waypoints):
        info = wp["mission_info"] or {}
        print(f"\nWaypoint {i+1}  |  sent at t={wp['send_time']:.3f}s")
        print(f"  Coordinates : x={info.get('x',0):.1f}  y={info.get('y',0):.1f}  z={info.get('z',0):.1f}")
        print(f"  Target sysid: {info.get('target_system','?')}")
        print(f"  Delivered   : {'YES' if wp['delivered'] else 'NO -- packet did not reach follower'}")
        print()

        if not wp["path"]:
            print("  [!] No captures contain this packet")
            continue

        # Print hop-by-hop
        print("  Hop-by-hop trace:")
        print(f"  {'Node':<10} {'Role':<12} {'Transmitter':<12} {'Next-hop MAC':<18} {'Timestamp'}")
        print(f"  {'-'*8:<10} {'-'*10:<12} {'-'*10:<12} {'-'*16:<18} {'-'*12}")

        seen_nodes = set()
        for hop in wp["path"]:
            n     = hop["seen_by_node"]
            role  = hop["seen_by_role"]
            ts    = hop["timestamp"]
            tx    = hop["tx_mac"] or "?"
            rx    = hop["rx_mac"] or "?"
            tx_n  = NODE_ROLES.get(hop["tx_node"], "?") if hop["tx_node"] is not None else "?"

            marker = ""
            if n not in seen_nodes:
                seen_nodes.add(n)
                marker = " <-- first seen here"

            print(f"  {f'Node {n}':<10} {role:<12} {tx_n:<12} {rx:<18} t={ts:.3f}s{marker}")

        # Summarise path
        nodes_hit = sorted(set(h["seen_by_node"] for h in wp["path"]))
        path_str  = " -> ".join(
            f"{NODE_ROLES.get(n, f'Node{n}')} ({NODE_IPS.get(n,'?')})"
            for n in nodes_hit
        )
        print(f"\n  Path: {path_str}")

        missing = [n for n in range(4) if n not in nodes_hit]
        if missing:
            missing_str = ", ".join(
                f"Node {n} ({NODE_ROLES.get(n,'?')})" for n in missing
            )
            print(f"  [!] Not seen at: {missing_str}")

    print(f"\n{SEP}")
    delivered = sum(1 for wp in waypoints if wp["delivered"])
    print(f"  Summary: {delivered}/{len(waypoints)} waypoints delivered to follower")
    print(SEP)


# ── Argument parsing ──────────────────────────────────────────────────────────

def parse_args():
    parser = argparse.ArgumentParser(
        description="Trace MISSION_ITEM packets through ns-3 multihop PCAP files"
    )
    parser.add_argument(
        "--dir", "-d",
        help="Directory containing PCAP files (auto-detects node index from filename)"
    )
    parser.add_argument(
        "--files", "-f",
        nargs="+",
        help="PCAP files in node order: node0.pcap node1.pcap node2.pcap node3.pcap"
    )
    return parser.parse_args()


def resolve_pcap_files(args):
    """Returns dict: node_index -> filepath"""
    if args.files:
        return {i: f for i, f in enumerate(args.files)}

    if args.dir:
        pattern = os.path.join(args.dir, "*.pcap")
        found = sorted(glob.glob(pattern))
        if not found:
            print(f"[ERROR] No PCAP files found in {args.dir}")
            sys.exit(1)

        # Auto-detect node index from filename pattern *-N-0.pcap
        result = {}
        for f in found:
            base = os.path.basename(f)
            parts = base.replace(".pcap", "").split("-")
            for j, part in enumerate(parts):
                if part.isdigit() and j > 0:
                    node_idx = int(part)
                    if node_idx not in result:
                        result[node_idx] = f
                        break
        if result:
            return result

        # Fallback: assign in sorted order
        return {i: f for i, f in enumerate(found)}

    # Default: look in current directory
    found = sorted(glob.glob("*.pcap"))
    if not found:
        print("[ERROR] No PCAP files found. Use --dir or --files.")
        sys.exit(1)
    return {i: f for i, f in enumerate(found)}


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    args = parse_args()
    pcap_files = resolve_pcap_files(args)

    print(f"\nLoading {len(pcap_files)} PCAP file(s):")
    for node_idx, path in sorted(pcap_files.items()):
        print(f"  Node {node_idx} ({NODE_ROLES.get(node_idx, '?')}): {os.path.basename(path)}")

    node_events = load_pcaps(pcap_files)
    waypoints   = build_travel_path(node_events)

    if not waypoints:
        print("\n[!] No MISSION_ITEM packets found in any PCAP file.")
        print("    Check that udp.dstport == 20000 traffic exists.")
        sys.exit(0)

    print_report(waypoints)


if __name__ == "__main__":
    main()
