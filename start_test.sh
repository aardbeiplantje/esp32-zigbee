#!/bin/bash

uid=$(id -u)
socat_opt=
tgt_ipv6="[fe80::eeda:3bff:febf:9400%wlan0]"
tgt_ipv4="192.168.1.80"

# ipv6 TCP IN+OUT
tcp_port=56765
uart_fn=/run/user/$uid/uart_${tcp_port}_tcp_ipv6
socat -b8 ${socat_opt} pty,link=${uart_fn},raw,unlink-close=0 TCP6-LISTEN:${tcp_port},ipv6only=1,reuseaddr,fork &
sleep 5
stty -F ${uart_fn} 115200 cs8 -parenb -cstopb raw -echo
cat ${uart_fn} &
echo "$uart_fn"

# ipv4 TCP IN+OUT
tcp_port=56765
uart_fn=/run/user/$uid/uart_${tcp_port}_tcp_ipv4
socat -b8 ${socat_opt} pty,link=${uart_fn},raw,unlink-close=0 TCP4-LISTEN:${tcp_port},reuseaddr,fork &
sleep 5
stty -F ${uart_fn} 115200 cs8 -parenb -cstopb raw -echo
cat ${uart_fn} &
echo "$uart_fn"

# ipv6 UDP IN/OUT
udp_port=56765
uart_fn=/run/user/$uid/uart_${udp_port}_udp_ipv6
socat -b8 ${socat_opt} UDP6:${tgt_ipv6}:${udp_port},ipv6only=1,reuseaddr pty,link=${uart_fn},raw,unlink-close=0 &
sleep 5
stty -F ${uart_fn} 115200 cs8 -parenb -cstopb raw -echo
cat ${uart_fn} &
echo "$uart_fn"

# upv4 UDP IN/OUT
tgt_ipv4=192.168.1.80
uart_fn=/run/user/$uid/uart_${udp_port}_udp_ipv4
socat -b8 ${socat_opt} UDP4:${tgt_ipv4}:${udp_port} pty,link=${uart_fn},raw,unlink-close=0 &
sleep 5
stty -F ${uart_fn} 115200 cs8 -parenb -cstopb raw -echo
cat ${uart_fn} &
echo "$uart_fn"

# ipv6 IN only
udp_in_port=56965
socat ${socat_opt} UDP6-LISTEN:$udp_in_port,ipv6only=1,reuseaddr STDOUT &

# ipv4 IN only
socat ${socat_opt} UDP4-LISTEN:$udp_in_port STDOUT &

# ipv6 OUT only
udp_out_port=56865
uart_in_fn=/run/user/$uid/uart_${udp_out_port}_udp_ipv6_out
socat -b8 ${socat_opt} pty,link=${uart_in_fn},raw,unlink-close=0 UDP6-SENDTO:$tgt_ipv6:$udp_out_port,ipv6only=1 &
echo "$uart_in_fn"

# ipv4 OUT only
uart_in_fn=/run/user/$uid/uart_${udp_out_port}_udp_ipv4_out
socat -b8 ${socat_opt} pty,link=${uart_in_fn},raw,unlink-close=0 UDP-SENDTO:$tgt_ipv4:$udp_out_port &
echo "$uart_in_fn"


wait
