# UDP to MQTT bidirectional bridge

# Configuration
LISTEN_UDP_IP = "0.0.0.0"
LISTEN_UDP_PORT = 3000
TALK_UDP_IP = "10.253.140.255"
TALK_UDP_PORT = LISTEN_UDP_PORT # this can be a different number
MQTT_BROKER = "e269b8c9277140d581347bd86428027f.s1.eu.hivemq.cloud"#"odu-mqtt-broker-788a5a30.a02.usw2.aws.hivemq.cloud"
MQTT_PORT = 8883
MQTT_TOPIC = "tunnel"
MQTT_TOPIC_P = "poisontunnel"

import os
import socket
import struct
import uuid
from paho import mqtt as pmqtt
import paho.mqtt.client as mqtt
from paho.mqtt.properties import Properties
from paho.mqtt.packettypes import PacketTypes
from paho.mqtt.subscribeoptions import SubscribeOptions
from scapy.all import Ether, IP, UDP, Raw, wrpcap

# Get MAC address of the local interface
def get_mac_address():
    mac = uuid.getnode()
    return ':'.join(("%012X" % mac)[i:i+2] for i in range(0, 12, 2))

def on_publish(client, userdata, mid):
    print("fMessage {mid} published to MQTT")          

my_mac_address = get_mac_address()

# Get local IP addresses
local_addresses = []
hostname = socket.gethostname()
my_ip_address='10.253.140.55'
local_addresses.append(my_ip_address)

# UDP Socket setup
udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
#udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
udp_sock.bind((LISTEN_UDP_IP, LISTEN_UDP_PORT))

# Multicast setup
#udp_sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 32)
#mreq = struct.pack("4sl", socket.inet_aton(TALK_UDP_IP), socket.INADDR_ANY)
#udp_sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

# MQTT Client setup
def on_connect(client, userdata, flags, rc, properties=None):
    print("Connected to MQTT broker with result code:",str(rc))
    # turn on noLocal, so we don't get our own dogfood back
    options = SubscribeOptions(noLocal=True)
    client.subscribe(MQTT_TOPIC, options=options)
    client.subscribe(MQTT_TOPIC_P, options=options)

def on_message(client, userdata, msg, properties=None):
    print(f"Message received from MQTT topic {msg.topic}")
    udp_sock.sendto(msg.payload, (TALK_UDP_IP, TALK_UDP_PORT))

# set up MQTT connection
mqtt_client = mqtt.Client(protocol=mqtt.MQTTv5)
mqtt_client.on_connect = on_connect
mqtt_client.on_message = on_message
mqtt_client.on_publish = on_publish

# Enable TLS for secure connection
mqtt_client.tls_set(tls_version=pmqtt.client.ssl.PROTOCOL_TLS)
# Set username and password
mqtt_client.username_pw_set("admin", "Admin123")

mqtt_client.connect(MQTT_BROKER, MQTT_PORT, keepalive=60,
                 clean_start=mqtt.MQTT_CLEAN_START_FIRST_ONLY, properties=None)
mqtt_client.loop_start()

# Main loop to receive UDP packets and forward to MQTT
try:
    pcap_file = 'captured_packets.pcap'
    while True:
        data, addr = udp_sock.recvfrom(2500)
        ip_address, port = addr
        
        # Create a packet using the received data with proper fields
        packet = Ether(src=my_mac_address, dst="ff:ff:ff:ff:ff:ff") / IP(src=LISTEN_UDP_IP, dst=ip_address) / UDP(sport=LISTEN_UDP_PORT, dport=port) / Raw(load=data)
        
        # Write the packet to the pcap file
        wrpcap(pcap_file, [packet], append=True)
        print(f"Packet from {ip_address}:{port} written to {pcap_file}")
        print(f"Received UDP packet from {addr}")
        
        # Ignore messages from local addresses
        if ip_address in local_addresses and port == LISTEN_UDP_PORT:
            print("ignoring packet")
            pass
        else:
            print("publishing")
            mqtt_client.publish(MQTT_TOPIC, data)
            mqtt_client.publish(MQTT_TOPIC_P, data)
except KeyboardInterrupt:
    print("Exiting...")
finally:
    udp_sock.close()
    mqtt_client.loop_stop()
    mqtt_client.disconnect()
