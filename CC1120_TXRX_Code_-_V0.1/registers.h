#define IOCFG3        0x0000
#define IOCFG2        0x0001
#define IOCFG1        0x0002
#define IOCFG0        0x0003
#define SYNC3         0x0004
#define SYNC2         0x0005
#define SYNC1         0x0006
#define SYNC0         0x0007
#define SYNC_CFG1     0x0008
#define SYNC_CFG0     0x0009
#define DEVIATION_M   0x000A
#define MODCFG_DEV_E  0x000B
#define DCFILT_CFG    0x000C
#define PREAMBLE_CFG1 0x000D
#define PREAMBLE_CFG0 0x000E
#define FREQ_IF_CFG   0x000F
#define IQIC          0x0010
#define CHAN_BW       0x0011
#define MDMCFG1       0x0012
#define MDMCFG0       0x0013

#define RXFIRST           0xD2
#define TXFIRST           0xD3
#define RXLAST            0xD4
#define TXLAST            0xD5
#define NUM_TXBYTES       0xD6
#define NUM_RXBYTES       0xD7
#define FIFO_NUM_TXBYTES  0xD8
#define FIFO_NUM_RXBYTES  0xD9
 
// The following are the states
#define STATEIDLE   B000
#define STATERX     B001
#define STATETX     B010
#define STATERXERR  B110
#define STATETXERR  B111
