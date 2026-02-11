#!/bin/bash

IF=wlan0
IPV6PR=${1:-fd00:1234:5678:9abc}
IPV6AD=${2:-"${IPV6PR}::1/64 ${IPV6PR}::2/64"}
IPV6GW=${3:-fe80::1}

sysctl net.ipv6.conf.$IF.disable_ipv6=0
sysctl net.ipv6.conf.$IF.addr_gen_mode=0
sysctl net.ipv6.conf.$IF.accept_ra=1
sysctl net.ipv6.conf.$IF.accept_ra_defrtr=1
sysctl net.ipv6.conf.$IF.accept_ra_pinfo=1
ip address flush dev ${IF}
ip link set dev ${IF} down
ip link set dev ${IF} up
ip route add fe80::/64 via $IPV6GW dev ${IF} metric 100 pref high onlink
ip route add ::/0 via $IPV6GW dev ${IF} metric 100 pref high onlink
ip route add ${IPV6PR}::/64 dev ${IF} metric 90 pref high onlink
ip route replace fe80::/64 dev ${IF} metric 100 pref high onlink
ip route replace fd00::/64 dev ${IF} metric 100 pref high onlink
ip route replace ::/0 via $IPV6GW dev ${IF} metric 100 pref high onlink
for i in ${IPV6AD}; do
  ip address add $i dev ${IF}
done
ip link set dev ${IF} up
ip -6 route list
