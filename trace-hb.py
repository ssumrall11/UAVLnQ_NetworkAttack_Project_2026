"""
trace_heartbeat_path.py
-----------------------
Traces HEARTBEAT (MAVLink msgid=0) packets across ns-3 node PCAPs to show
which nodes the packet passed through, supporting any number of nodes and
bidirectional traffic (N0->Nlast and Nlast->N0).

Fixes WiFi promiscuous capture duplicates by only counting packets where
the node is the actual intended destination (ip.dst == node_ip).

Usage:
    python3 trace_heartbeat_path.py --dir /path/to/pcap/dir --nNodes 4
    python3 trace_heartbeat_path.py --files node0.pcap node1.pcap ... nodeN.pcap
    python3 trace_heartbeat_path.py --dir /path/to/pcaps --nNodes 7

For wormhole topology:
    python3 trace_heartbeat_path.py \
        --files main-0-0.pcap wormhole-W1-7-2.pcap wormhole-W2-8-2.pcap main-6-0.pcap \
        --roles "Leader" "W1" "W2" "Follower" \
        --leader-ip 10.1.1.1 --follower-ip 10.1.1.7
"""

import argparse
import struct
import os
import glob
import sys

try:
    from scapy.all import rdpcap, UDP, IP
except ImportError:
    print("[ERROR] scapy not installed. Run: pip install scapy --break-system-packages")
    sys.exit(1)

MAVLINK_PORT             = 20000
MAVLINK_MSG_ID_HEARTBEAT = 0

MAV_TYPE_NAMES = {
    0: "GENERIC", 1: "FIXED_WING", 2: "QUADROTOR", 3: "COAXIAL",
    4: "HELICOPTER", 6: "GCS", 10: "FLAPPING_WING", 13: "HEXAROTOR",
    14: "OCTOROTOR", 15: "TRICOPTER", 27: "ONBOARD_CONTROLLER",
}
MAV_AUTOPILOT_NAMES = {
    0: "GENERIC", 3: "ARDUPILOTMEGA", 8: "INVALID", 18: "PX4",
}
MAV_STATE_NAMES = {
    0: "UNINIT", 1: "BOOT", 2: "CALIBRATING", 3: "STANDBY",
    4: "ACTIVE", 5: "CRITICAL", 6: "EMERGENCY", 7: "POWEROFF",
}


# ---------------------------------------------------------------------------
# MAVLink decode — supports v1 (0xFE) and v2 (0xFD)
# ---------------------------------------------------------------------------
def decode_heartbeat(raw_payload):
    if len(raw_payload) < 6:
        return None
    magic = raw_payload[0]

    if magic == 0xFE:                          # MAVLink v1
        plen  = raw_payload[1]
        seq   = raw_payload[2]
        sysid = raw_payload[3]
        compid= raw_payload[4]
        msgid = raw_payload[5]
        if msgid != MAVLINK_MSG_ID_HEARTBEAT:
            return None
        payload = raw_payload[6:6 + plen]
        if len(payload) < 9:
            return None
        custom_mode, = struct.unpack_from('<I', payload, 0)
        return dict(version=1, mavlink_seq=seq, sysid=sysid, compid=compid,
                    type=payload[4], autopilot=payload[5], base_mode=payload[6],
                    system_status=payload[7], custom_mode=custom_mode)

    elif magic == 0xFD:                        # MAVLink v2
        if len(raw_payload) < 10:
            return None
        plen  = raw_payload[1]
        seq   = raw_payload[4]
        sysid = raw_payload[5]
        compid= raw_payload[6]
        msgid = raw_payload[7] | (raw_payload[8] << 8) | (raw_payload[9] << 16)
        if msgid != MAVLINK_MSG_ID_HEARTBEAT:
            return None
        payload = raw_payload[10:10 + plen]
        if len(payload) < 9:
            return None
        custom_mode, = struct.unpack_from('<I', payload, 0)
        return dict(version=2, mavlink_seq=seq, sysid=sysid, compid=compid,
                    type=payload[4], autopilot=payload[5], base_mode=payload[6],
                    system_status=payload[7], custom_mode=custom_mode)

    return None


# ---------------------------------------------------------------------------
# Load PCAPs
# Only counts packets where ip.dst == this node's own IP to eliminate
# WiFi ad-hoc promiscuous capture duplicates at relay nodes.
# Returns {node_idx: [(t, src_ip, dst_ip, hb_info), ...]}
# ---------------------------------------------------------------------------
def load_pcaps(pcap_files, node_ips):
    node_events = {}
    for node_idx, path in sorted(pcap_files.items()):
        events     = []
        node_ip    = node_ips.get(node_idx)   # this node's own IP

        try:
            pkts = rdpcap(path)
        except Exception as e:
            print(f"  [WARN] Could not read {path}: {e}")
            continue

        for p in pkts:
            if not (IP in p and UDP in p):
                continue
            if p[UDP].dport != MAVLINK_PORT:
                continue

            # --- Key fix: skip packets not destined for this node ---
            # This eliminates WiFi promiscuous duplicates at relay nodes
            if node_ip and p[IP].dst != node_ip:
                continue

            raw  = bytes(p[UDP].payload)
            info = decode_heartbeat(raw)
            if info is None:
                continue

            events.append((float(p.time), p[IP].src, p[IP].dst, info))

        node_events[node_idx] = sorted(events, key=lambda e: e[0])
        print(f"  Node {node_idx}: {len(events)} heartbeat packet(s) found"
              + (f"  [{node_ip}]" if node_ip else ""))

    return node_events


# ---------------------------------------------------------------------------
# Cluster heartbeats by send time into forward / reverse flows
# ---------------------------------------------------------------------------
def cluster_heartbeats(node_events, node_ips, leader_ip, follower_ip,
                       time_tolerance=0.5):
    """
    Returns (forward_list, reverse_list).
    Each entry: {send_time, direction, path, delivered, hb_info}
    """
    ip_to_node = {v: k for k, v in node_ips.items()}

    def get_send_times(source_ip):
        times = []
        for events in node_events.values():
            for (t, src, dst, info) in events:
                if src == source_ip:
                    times.append(t)
        times.sort()
        clustered = []
        for t in times:
            if not clustered or t - clustered[-1] > time_tolerance:
                clustered.append(t)
        return clustered

    forward_times = get_send_times(leader_ip)
    reverse_times = get_send_times(follower_ip)

    def build(send_times, src_ip, dst_ip, direction):
        result = []
        dest_node = ip_to_node.get(dst_ip)

        for send_time in send_times:
            hb = dict(send_time=send_time, direction=direction,
                      path=[], delivered=False, hb_info=None)

            for node_idx in sorted(node_events.keys()):
                for (t, s, d, info) in node_events[node_idx]:
                    if abs(t - send_time) > time_tolerance:
                        continue
                    if s != src_ip:
                        continue
                    hb["path"].append(dict(node=node_idx, timestamp=t,
                                           src_ip=s, dst_ip=d))
                    if hb["hb_info"] is None:
                        hb["hb_info"] = info
                    if dest_node is not None and node_idx == dest_node:
                        hb["delivered"] = True
                    break   # one entry per node per heartbeat

            result.append(hb)
        return result

    forward = build(forward_times, leader_ip,   follower_ip, "forward (N0 → Nlast)")
    reverse = build(reverse_times, follower_ip, leader_ip,   "reverse (Nlast → N0)")
    return forward, reverse


# ---------------------------------------------------------------------------
# Print report
# ---------------------------------------------------------------------------
def print_report(heartbeats, node_roles, node_ips, label):
    SEP = "=" * 65
    print(f"\n{SEP}")
    print(f"  {label}")
    print(SEP)

    for i, hb in enumerate(heartbeats):
        info      = hb["hb_info"] or {}
        mav_type  = MAV_TYPE_NAMES.get(info.get("type", -1),
                                        f"type={info.get('type','?')}")
        autopilot = MAV_AUTOPILOT_NAMES.get(info.get("autopilot", -1),
                                             f"ap={info.get('autopilot','?')}")
        status    = MAV_STATE_NAMES.get(info.get("system_status", -1),
                                         f"status={info.get('system_status','?')}")

        print(f"\nHeartbeat {i+1}  [{hb['direction']}]  t={hb['send_time']:.3f}s")
        print(f"  MAVLink v{info.get('version','?')}  "
              f"sysid={info.get('sysid','?')}  seq={info.get('mavlink_seq','?')}")
        print(f"  Type: {mav_type}  Autopilot: {autopilot}  Status: {status}")
        print(f"  Delivered: {'YES' if hb['delivered'] else 'NO -- packet dropped/lost'}")

        if not hb["path"]:
            print("  [!] No captures contain this packet at its destination")
            continue

        print()
        print(f"  {'Node':<8} {'Role':<14} {'Src IP':<14} {'Dst IP':<14} {'Time'}")
        print(f"  {'-'*6:<8} {'-'*12:<14} {'-'*12:<14} {'-'*12:<14} {'-'*10}")
        for hop in hb["path"]:
            n    = hop["node"]
            role = node_roles.get(n, f"Node{n}")
            print(f"  {f'Node {n}':<8} {role:<14} {hop['src_ip']:<14} "
                  f"{hop['dst_ip']:<14} t={hop['timestamp']:.3f}s")

        nodes_hit = [h["node"] for h in hb["path"]]
        path_str  = " -> ".join(
            f"{node_roles.get(n, f'Node{n}')} ({node_ips.get(n,'?')})"
            for n in nodes_hit
        )
        print(f"\n  Path: {path_str}")

    print(f"\n{SEP}")
    delivered = sum(1 for hb in heartbeats if hb["delivered"])
    if heartbeats:
        print(f"  Summary: {delivered}/{len(heartbeats)} heartbeats delivered")
        print(f"  PDR    : {delivered/len(heartbeats):.3f}")
    else:
        print("  No heartbeats in this direction.")
    print(SEP)


# ---------------------------------------------------------------------------
# Argument parsing
# ---------------------------------------------------------------------------
def parse_args():
    parser = argparse.ArgumentParser(
        description="Trace bidirectional HEARTBEAT packets through ns-3 PCAP files"
    )
    parser.add_argument("--dir",         "-d", help="Directory containing PCAP files")
    parser.add_argument("--files",       "-f", nargs="+",
                        help="PCAP files in node order")
    parser.add_argument("--nNodes",      "-n", type=int, default=0,
                        help="Number of drone nodes (auto-detected if 0)")
    parser.add_argument("--roles",       "-r", nargs="+",
                        help="Role labels per node e.g. Leader Relay1 Relay2 Follower")
    parser.add_argument("--base-ip",           default="10.1.1",
                        help="IP prefix (default: 10.1.1)")
    parser.add_argument("--leader-ip",         default=None,
                        help="Override leader IP (e.g. 10.1.1.1)")
    parser.add_argument("--follower-ip",       default=None,
                        help="Override follower IP (e.g. 10.1.1.7)")
    parser.add_argument("--node-ips",          nargs="+", default=None,
                        help="IP per node in same order as --files "
                             "e.g. 10.1.1.1 10.1.1.8 10.1.1.9 10.1.1.7")
    return parser.parse_args()


def resolve_pcap_files(args):
    if args.files:
        return {i: f for i, f in enumerate(args.files)}

    search_dir = args.dir or "."
    pattern    = os.path.join(search_dir, "*.pcap")
    found      = sorted(glob.glob(pattern))
    if not found:
        print(f"[ERROR] No PCAP files found in {search_dir}")
        sys.exit(1)

    # Extract node index from filename (e.g. multihop-aodv20260611-0-0.pcap)
    result = {}
    for f in found:
        base  = os.path.basename(f)
        parts = base.replace(".pcap", "").split("-")
        for j in range(len(parts) - 1, 0, -1):
            if parts[j].isdigit():
                idx = int(parts[j])
                if idx not in result:
                    result[idx] = f
                break

    if result:
        return result

    return {i: f for i, f in enumerate(found)}


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    args       = parse_args()
    pcap_files = resolve_pcap_files(args)

    n_nodes = args.nNodes if args.nNodes > 0 else len(pcap_files)
    base    = args.base_ip

    # Build node IP map: node_index -> IP
    node_ips = {i: f"{base}.{i+1}" for i in range(n_nodes)}
    for idx in pcap_files:
        if idx not in node_ips:
            node_ips[idx] = f"{base}.{idx+1}"

    # Build role map
    if args.roles:
        node_roles = {i: r for i, r in enumerate(args.roles)}
    else:
        all_indices = sorted(pcap_files.keys())
        node_roles  = {}
        for i, idx in enumerate(all_indices):
            if i == 0:
                node_roles[idx] = "Leader"
            elif i == len(all_indices) - 1:
                node_roles[idx] = "Follower"
            else:
                node_roles[idx] = f"Relay {i}"

    sorted_keys = sorted(pcap_files.keys())
    leader_ip   = args.leader_ip   or node_ips.get(sorted_keys[0],  f"{base}.1")
    follower_ip = args.follower_ip or node_ips.get(sorted_keys[-1], f"{base}.{n_nodes}")

    print(f"\nLoading {len(pcap_files)} PCAP file(s):")
    for idx, path in sorted(pcap_files.items()):
        role = node_roles.get(idx, f"Node{idx}")
        ip   = node_ips.get(idx, "?")
        print(f"  Node {idx} ({role}) [{ip}]: {os.path.basename(path)}")

    print(f"\nLeader IP  : {leader_ip}")
    print(f"Follower IP: {follower_ip}")
    print(f"Note: Only counting packets where ip.dst == node's own IP")
    print(f"      (eliminates WiFi ad-hoc promiscuous capture duplicates)\n")

    node_events = load_pcaps(pcap_files, node_ips)

    if not node_events:
        print("\n[!] No heartbeat packets found in any PCAP.")
        sys.exit(0)

    forward_hbs, reverse_hbs = cluster_heartbeats(
        node_events, node_ips, leader_ip, follower_ip)

    if not forward_hbs and not reverse_hbs:
        print("\n[!] No HEARTBEAT packets found.")
        print("    Check udp.dstport==20000 traffic exists.")
        print("    MAVLink magic must be 0xFE (v1) or 0xFD (v2).")
        sys.exit(0)

    if forward_hbs:
        print_report(forward_hbs, node_roles, node_ips,
                     "FORWARD HEARTBEATS  (Leader -> Follower)")

    if reverse_hbs:
        print_report(reverse_hbs, node_roles, node_ips,
                     "REVERSE HEARTBEATS  (Follower -> Leader)")

    # Combined summary
    SEP       = "=" * 65
    total_tx  = len(forward_hbs) + len(reverse_hbs)
    fwd_rx    = sum(1 for h in forward_hbs if h["delivered"])
    rev_rx    = sum(1 for h in reverse_hbs if h["delivered"])
    total_rx  = fwd_rx + rev_rx

    print(f"\n{SEP}")
    print("  COMBINED SUMMARY")
    print(SEP)
    if forward_hbs:
        print(f"  Forward  TX={len(forward_hbs)}  RX={fwd_rx}  "
              f"PDR={fwd_rx/len(forward_hbs):.3f}")
    else:
        print("  Forward  No heartbeats")
    if reverse_hbs:
        print(f"  Reverse  TX={len(reverse_hbs)}  RX={rev_rx}  "
              f"PDR={rev_rx/len(reverse_hbs):.3f}")
    else:
        print("  Reverse  No heartbeats")
    if total_tx:
        print(f"  Overall  TX={total_tx}  RX={total_rx}  "
              f"PDR={total_rx/total_tx:.3f}")
    print(SEP)


if __name__ == "__main__":
    main()
