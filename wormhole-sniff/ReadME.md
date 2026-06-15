### Wormhole Attack + Packet Drop on AODV Multihop Topology integrated into UAVLnQ
---
By Sarah Sumrall

Assisted by Abdelrahman Yousef

Supervised by Dr. Guy Gogniat and Dr. Maria Mendez-Real

Based off of: HLM2022, “HLM2022/Wormhole-for-NS3: Source code to create a network with a wormhole in NS-3,” GitHub, 2025. Available: https://github.com/HLM2022/Wormhole-for-NS3/tree/main

---
This is the first wormhole attack implementation without any additional attack logic. This implementtation can be run in NS-3 and UAVLnQ. In this directory you have the Cmake file along with the main.cc file which not only helps you build and run in ns-3 but also connects to UAVLnQ entirely. Right now it only works if the drones are statonary (no movement). 

In the test-logs sub directory you have:
- Telemetry logs (.csv)
- Network Traffic Logs (.pcap)
- Routing Tables (.routes/.xml/.txt)
- Netanim animation (anim...xml)
- NS-3 Logs with FlowMonitor Performance Statistics (.log)

The logs with the designation: UAVLn@, are for the implementations integrated entirely into UAVLnQ.

The logs with the designation: NS3, are for the implementations tested with NS-3.

You can use the python scripts for extra verfication on the heartbeat messages by doing these scripts in the python virtual environment (source .venv/bin/activate in directory UAVLnQ)

```json
python - <<'PY'
from scapy.all import rdpcap,IP,UDP

pkts=rdpcap("ns3-output/wormhole-aodv-20260611_112401-main-0-0.pcap")

for p in pkts:
    if IP in p and UDP in p and p[UDP].dport==20000:
        print(p[IP].src,"->",p[IP].dst)
PY
```
```json
from scapy.all import rdpcap,IP,UDP

for fn in [                                                          
"ns3-output/wormhole-aodv-20260611_112401-main-0-0.pcap",
"ns3-output/wormhole-aodv-20260611_112401-wormhole-W1-7-2.pcap",
"ns3-output/wormhole-aodv-20260611_112401-wormhole-W2-8-2.pcap",
"ns3-output/wormhole-aodv-20260611_112401-main-6-0.pcap"
]:
    flows=set()

    for p in rdpcap(fn):
        if IP in p and UDP in p and p[UDP].dport==20000:
            flows.add((p[IP].src,p[IP].dst))

    print("\n",fn)
    for s,d in sorted(flows):
        print(" ",s,"->",d)
PY
```

To run the simulation in only NS-3. You use the command: 

```NS_LOG="DroneWormholeMesh=info" ~/ns-3-dev/build/scratch/multihop-with-attacker-sniff/ns3-dev-multihop-with-attacker-sniff-default --nNodes=7 --simTime=200 --o=/home/ubuntu/UAVLnQ/ns3-output/ 2>&1 | tee /home/ubuntu/UAVLnQ/ns3-output/wh.log```

To run the simulation in UAVLnQ, you use change the drones-config file to the correct parameters (as seen in directions in the main rREADME and INSTALLATION guide) and use these commands:
```json
flatpak run --share=network org.mavlink.qgroundcontrol

sim_vehicle.py -v ArduCopter -I0 --sysid=1 --console --out=udp:127.0.0.1:14551 --out=udp:127.0.0.1:14552 --out=udp:127.0.0.1:14553 --custom-location=-35.3628,149.16500,0,0

sim_vehicle.py -v ArduCopter -I1 --sysid=2 --console --out=udp:127.0.0.1:14561 --out=udp:127.0.0.1:14562 --out=udp:127.0.0.1:14563 --custom-location=-35.3628,149.16588,0,0

sim_vehicle.py -v ArduCopter -I2 --sysid=3 --console --out=udp:127.0.0.1:14571 --out=udp:127.0.0.1:14572 --out=udp:127.0.0.1:14573 --custom-location=-35.3628,149.16676,0,0

sim_vehicle.py -v ArduCopter -I3 --sysid=4 --console --out=udp:127.0.0.1:14581 --out=udp:127.0.0.1:14582 --out=udp:127.0.0.1:14583 --custom-location=-35.3628,149.16764,0,0

sim_vehicle.py -v ArduCopter -I4 --sysid=5 --console --out=udp:127.0.0.1:14591 --out=udp:127.0.0.1:14592 --out=udp:127.0.0.1:14593 --custom-location=-35.3628,149.16852,0,0

sim_vehicle.py -v ArduCopter -I5 --sysid=6 --console --out=udp:127.0.0.1:14601 --out=udp:127.0.0.1:14602 --out=udp:127.0.0.1:14603 --custom-location=-35.3628,149.16940,0,0

sim_vehicle.py -v ArduCopter -I6 --sysid=7 --console --out=udp:127.0.0.1:14611 --out=udp:127.0.0.1:14612 --out=udp:127.0.0.1:14613 --custom-location=-35.3628,149.17028,0,0

cd UAVLnQ

source .venv/bin/activate

python connect.py 

source .venv/bin/activate

python Mavlink-NS3-Parser.py
```

The default amount of nodes is 7. You can change it in main.cc here:

```uint32_t nNodes = 7;``

The default distance btween nodes is 80 meters. You can change the distance between nodes in the main.cc file here:

```for (uint32_t i = 0; i < nNodes; i++) pos->Add(Vector(i * 80, 0, 20));``

The default maximum transmission range is 82 meters (this always has to be bigger than the max distance). You can set the maximum transmission range for the WIFI in the main.cc file here:

```channel.AddPropagationLoss("ns3::RangePropagationLossModel", "MaxRange", DoubleValue(82));```

The default number of heartbeat messages is 4 for each direction (N3-N0 and N0-N3). You can change the number of heartbeat messages here: 

```Simulator::Schedule(Seconds(2.0),  &SendHeartbeatFromDrone0);```

```Simulator::Schedule(Seconds(16.0), &SendHeartbeatFromLastDrone);```
