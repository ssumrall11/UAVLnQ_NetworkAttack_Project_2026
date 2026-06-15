### AODV Multihop Baseline Topology integrated into UAVLnQ
---
By Sarah Sumrall

Assisted by Abdelrahman Yousef

Supervised by Dr. Guy Gogniat and Dr. Maria Mendez-Real

---

This is a baseline implementation in efforts to build a wormhole attack. Wormhole attacks targets routing protocols such as AODV and their multihop topologies. In this directory you have the Cmake file along with the main.cc file which not only helps you build and run in ns-3 but also connects to UAVLnQ entirely. Right now it only works if the drones are statonary (no movement). 

In the test-logs sub directory you have:
- Telemetry logs (.csv)
- Network Traffic Logs (.pcap)
- Routing Tables (.routes/.xml/.txt)
- Netanim animation (anim...xml)
- NS-3 Logs with FlowMonitor Performance Statistics (.log)

The logs with the designation: UAVLnQ, are for the implementations integrated entirely into UAVLnQ.

The logs with the designation: NS3, are for the implementations tested with NS-3.

You can use the python script: trace-hb.py to verify that the heartbeats are being sent correctly. (Only use it for baseline)

To run trace-hb.py you run:
```json
cd UAVLnQ

source .venv/bin/activate

python trace-py --files put/your/directory/here/simulation.pcap (put all your simulation pcaps here to track the number of heartbeat messages)
```

To run the simulation in only NS-3. You use the command: 
```NS_LOG="DroneZmqMeshNetwork=info:AodvRoutingProtocol=debug:Ipv4L3Protocol=info:ArpL3Protocol=info" ~/ns-3-dev/build/scratch/multihop-aodv/ns3-dev-multihop-aodv-default --nNodes=4 --simTime=100 --o=/home/ubuntu/UAVLnQ/ns3-output 2>&1 | tee /home/ubuntu/UAVLnQ/ns3-output/baseline-aodv_normal.log```


To run the simulation in UAVLnQ, you use change the drones-config file to the correct parameters (as seen in directions in the main rREADME and INSTALLATION guide) and use these commands:
```json
flatpak run --share=network org.mavlink.qgroundcontrol

sim_vehicle.py -v ArduCopter -I0 --sysid=1 --console --out=udp:127.0.0.1:14551 --out=udp:127.0.0.1:14552 --out=udp:127.0.0.1:14553 --custom-location=-35.3628,149.16500,0,0

sim_vehicle.py -v ArduCopter -I1 --sysid=2 --console --out=udp:127.0.0.1:14561 --out=udp:127.0.0.1:14562 --out=udp:127.0.0.1:14563 --custom-location=-35.3628,149.16588,0,0

sim_vehicle.py -v ArduCopter -I2 --sysid=3 --console --out=udp:127.0.0.1:14571 --out=udp:127.0.0.1:14572 --out=udp:127.0.0.1:14573 --custom-location=-35.3628,149.16676,0,0

sim_vehicle.py -v ArduCopter -I3 --sysid=4 --console --out=udp:127.0.0.1:14581 --out=udp:127.0.0.1:14582 --out=udp:127.0.0.1:14583 --custom-location=-35.3628,149.16764,0,0

cd UAVLnQ

source .venv/bin/activate

python connect.py 

source .venv/bin/activate

python Mavlink-NS3-Parser.py
```

The default amount of nodes is 4. You can change it in main.cc here:

```uint32_t nNodes = 4;```

The default distance btween nodes is 80 meters. You can change the distance between nodes in the main.cc file here:

``for (uint32_t i = 0; i < nNodes; i++) pos->Add(Vector(i * 80, 0, 20));```

The default maximum transmission range is 82 meters (this always has to be bigger than the max distance). You can set the maximum transmission range for the WIFI in the main.cc file here:

```channel.AddPropagationLoss("ns3::RangePropagationLossModel", "MaxRange", DoubleValue(82));```

The default number of heartbeat messages is 3 for each direction (N3-N0 and N0-N3). You can change the number of heartbeat messages here: 

```Simulator::Schedule(Seconds(2.0),  &SendHeartbeatFromDrone0);```

```Simulator::Schedule(Seconds(16.0), &SendHeartbeatFromLastDrone);```
