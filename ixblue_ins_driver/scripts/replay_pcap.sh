#!/bin/bash

PCAP_FILE=$1

INTERFACE=$2

IP_ADDR=$(ip -f inet addr show $INTERFACE | sed -En -e 's/.*inet ([0-9.]+).*/\1/p')
MAC_ADDR=$(cat /sys/class/net/$INTERFACE/address)

echo "modifying pcap file with your network settings IP: $IP_ADDR MAC: $MAC_ADDR"

tcprewrite  --infile=$PCAP_FILE \
            --outfile=temp.pcap\
            --dstipmap=192.168.134.20:$IP_ADDR\
            --enet-dmac=$MAC_ADDR\
            --srcipmap=192.168.133.155:$IP_ADDR\
            --enet-smac=$MAC_ADDR\
            --fixcsum

echo "replaying the file on IP: $IP_ADDR with mac: $MAC_ADDR  (<ctrl>+c) to stop"

sudo udpreplay -i $INTERFACE temp.pcap -r -1

rm temp.pcap

#Destination Address: 192.168.10.120
