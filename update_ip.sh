#!/bin/bash

# Retrieve IP address
IP_ADDRESS=$(hostname -I | awk '{print $1}')
if [ -z "$IP_ADDRESS" ]; then
    echo "Error: Failed to retrieve IP address."
    exit 1
fi

# Retrieve netmask (extract the netmask corresponding to the IP address)
INTERFACE=$(ip -o addr show | grep "$IP_ADDRESS" | awk '{print $2}')
NETMASK=$(ip -o -f inet addr show $INTERFACE | awk '/inet/ {print $4}' | cut -d'/' -f2)
if [ -z "$NETMASK" ]; then
    echo "Error: Failed to retrieve netmask."
    exit 1
fi

# Calculate netmask from CIDR
function cidr_to_netmask() {
    local cidr=$1
    local mask=""
    local octets=$(( cidr / 8 ))
    local bits=$(( cidr % 8 ))

    for (( i=0; i<4; i++ )); do
        if [ $i -lt $octets ]; then
            mask+="255"
        elif [ $i -eq $octets ]; then
            mask+=$(( 256 - 2**(8-bits) ))
        else
            mask+="0"
        fi

        if [ $i -lt 3 ]; then
            mask+="."
        fi
    done
    echo "$mask"
}

NETMASK=$(cidr_to_netmask $NETMASK)
echo "Retrieved Netmask for IP $IP_ADDRESS: $NETMASK"

# Split the IP address by dots
IFS='.' read -r -a IP_PARTS <<< "$IP_ADDRESS"

# Split the netmask by dots
IFS='.' read -r -a NETMASK_PARTS <<< "$NETMASK"
echo "Netmask parts: ${NETMASK_PARTS[0]}, ${NETMASK_PARTS[1]}, ${NETMASK_PARTS[2]}, ${NETMASK_PARTS[3]}"

# Replace the IP address in include/rtps/config.h

echo "Running sed on include/rtps/config.h"
sed -i "s/[[:space:]]*[0-9]\{1,3\},[[:space:]]*[0-9]\{1,3\},[[:space:]]*[0-9]\{1,3\},[[:space:]]*[0-9]\{1,3\}[[:space:]]*}; \
\/\/ Needs to be set in lwipcfg.h too./\
${IP_PARTS[0]}, ${IP_PARTS[1]}, ${IP_PARTS[2]}, ${IP_PARTS[3]}};\
\/\/ Needs to be set in lwipcfg.h too./" \
include/rtps/config.h


# Replace the IP address and netmask in include/netif.h
sed -i 's/#define NETIF_IPADDR ".*"/#define NETIF_IPADDR "'$IP_ADDRESS'"/' include/netif.h
sed -i 's/#define NETIF_NETMASK ".*"/#define NETIF_NETMASK "'$NETMASK'"/' include/netif.h



# Display the result for confirmation
echo "Updated IP Address: $IP_ADDRESS"
echo "Updated include/rtps/config.h:"
grep -E  'Needs to be set in lwipcfg.h too.' include/rtps/config.h
echo "Updated include/netif.h:"
grep -E 'NETIF_IPADDR' include/netif.h
