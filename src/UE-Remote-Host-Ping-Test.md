Run the following two commands to the 10.10.1.2
// sudo ip neigh add
// this add UE with IP 7.0.0.2 to the ArP cache on the remote host
// the ArP cache maps the IP to the destination MAC
// the destination MAC is Edge serger ethernet interface MAC address
// every new UE needs to add one entry

sudo ip neigh add 7.0.0.2 lladdr 02:06:00:00:00:0d dev enp94s0f1np1
sudo ip route add 7.0.0.0/24 dev enp94s0f1np1 scope link src 10.10.1.2


