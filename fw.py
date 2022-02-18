import pylink

fw = open("firmware.bin","rb")
data = fw.read()
fw.close()

print("file_size = ")
print(len(data))
size = len(data)

jlink = pylink.JLink()

jlink.open(20210319)
jlink.set_tif(pylink.enums.JLinkInterfaces.SWD)
jlink.connect('Cortex-M4')
jlink.coresight_configure()
jlink.reset()


jlink.memory_write32(0x40020018, [0x8C9DAEBF])
jlink.memory_write32(0x40020018, [0x13141516])
jlink.memory_write32(0x40020000, [0x21])
jlink.memory_write32(0x40020018, [0x8C9DAEBF])
jlink.memory_write32(0x40020018, [0])

jlink.memory_write32(0x8000000, [0xFFFFFFFF])

sr = [0]
while (sr[0] & 0x1) == 0:
  sr = jlink.memory_read32(0x40020008, 1)

jlink.memory_write32(0x40020008, [0x1])

jlink.memory_write32(0x40020018, [0x8C9DAEBF])
jlink.memory_write32(0x40020018, [0x13141516])
jlink.memory_write32(0x40020000, [0x24])
jlink.memory_write32(0x40020018, [0x8C9DAEBF])
jlink.memory_write32(0x40020018, [0])

tmp = [0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff]
aligned_size = (size)&0xFFFFFFF8;

print(aligned_size)

for x in range(size):
  if (x >= aligned_size):
    tmp[x - aligned_size] = data[x]

x = 0
while x < size:
  if (x < aligned_size):
    jlink.memory_write32(0x4002000C, [data[x+3] << 24 | data[x+2] << 16 | data[x+1] << 8 | data[x+0]])
    jlink.memory_write32(0x40020010, [data[x+7] << 24 | data[x+6] << 16 | data[x+5] << 8 | data[x+4]])
  else:
    jlink.memory_write32(0x4002000C, [tmp[3] << 24 | tmp[2] << 16 | tmp[1] << 8 | tmp[0]])
    jlink.memory_write32(0x40020010, [tmp[7] << 24 | tmp[6] << 16 | tmp[5] << 8 | tmp[4]])
	
  jlink.memory_write32(0x8000000+x, [0xFFFFFFFF])
  
  sr = [0]
  while (sr[0] & 0x1) == 0:
    sr = jlink.memory_read32(0x40020008, 1)

  jlink.memory_write32(0x40020008, [0x1])
  x+=8
  
jlink.reset()
jlink.close()
