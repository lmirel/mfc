import socket
import sys
import struct
# pip3 install salsa20
from salsa20 import Salsa20_xor

#https://github.com/Nenkai/PDTools/blob/master/SimulatorInterface/SimulatorInterface.cs
SendDelaySeconds = 10

ReceivePort = 33740
SendPort = 33739

port = ReceivePort

if len(sys.argv) == 2:
    # Get "IP address of Server" and also the "port number" from
    ip = sys.argv[1]
else:
    print("Run like : python3 gt7racedata.py <playstation-ip>")
    exit(1)

# Create a UDP socket
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# Bind the socket to the port
server_address = ('0.0.0.0', port)
s.bind(server_address)
s.settimeout(10)

#https://github.com/Nenkai/PDTools/blob/master/PDTools.Crypto/SimulationInterface/SimulatorInterfaceCryptorGT7.cs
def salsa20_dec(dat):
  KEY = b'Simulator Interface Packet GT7 ver 0.0'
  oiv = dat[0x40:0x44]
  iv1 = int.from_bytes(oiv, byteorder='little') # Seed IV is always located there
  iv2 = iv1 ^ 0xDEADBEAF #// Notice DEADBEAF, not DEADBEEF
  """
  print("OIV: %d bytes" % len(oiv))
  print(' '.join(format(x, '02x') for x in oiv))
  print("IV1: %d bytes" % len(iv1.to_bytes(4, 'big')))
  print(' '.join(format(x, '02x') for x in iv1.to_bytes(4, 'big')))
  print("IV2: %d bytes" % len(iv2.to_bytes(4, 'big')))
  print(' '.join(format(x, '02x') for x in iv2.to_bytes(4, 'big')))
  """
  IV = bytearray()
  IV.extend(iv2.to_bytes(4, 'little'))
  IV.extend(iv1.to_bytes(4, 'little'))
  #print("IV: %d bytes" % len(IV))
  #print(' '.join(format(x, '02x') for x in IV))
  """
  // Magic should be "G7S0" when decrypted
  SpanReader sr = new SpanReader(data);
  int magic = sr.ReadInt32();
  if (magic != 0x47375330) // 0S7G - G7S0
  """
  ddata = Salsa20_xor(dat, bytes(IV), KEY[0:32])#.decode()
  #check magic number
  magic = int.from_bytes(ddata[0:4], byteorder='little')
  if magic != 0x47375330:
    return bytearray(b'')
  return ddata

def send_hb(s):
  #send HB
  send_data = 'A'
  s.sendto(send_data.encode('utf-8'), (ip, SendPort))
  print('send heartbeat')

send_hb(s)

print("Ctrl+C to exit the program")

pknt = 0
while True:
  try:
    data, address = s.recvfrom(4096)
    pknt = pknt + 1
    print("received: %d bytes" % len(data))
    #print(' '.join(format(x, '02x') for x in data))
    ddata = salsa20_dec(data)
    #print("decoded: %d bytes" % len(ddata))
    #print(' '.join(format(x, '02x') for x in ddata))
    if len(ddata) > 0:
      #https://github.com/Nenkai/PDTools/blob/master/PDTools.SimulatorInterface/SimulatorPacketGT7.cs
      #RPM: 15th 4byte ints
      rpm = struct.unpack('f', ddata[15*4:15*4+4])
      print('RPM %d' %(rpm))
    if pknt > 100:
      send_hb(s)
      pknt = 0
  except Exception as e:
    send_hb(s)
    pknt = 0
    pass

