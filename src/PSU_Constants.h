#define RETRY_COUNT (uint8_t)3


#define PSU_READ_REQUEST (uint16_t)0x120
#define PSU_WRITE_REQUEST (uint16_t)0x1C0
#define PSU_ACK 0x07F
#define PSU_READY_TO_RECEIVE 0x000
#define PSU_NO_DATA_ACK 0x073

#define BUS_IDLE_TIMER (uint32_t)60 //milliseconds
#define PSU_ANSWER_TIMEOUT (uint32_t)15 //milliseconds

#define PMU_ANSWER_DELAY (uint32_t)5

#define PMU_READ_WRITE_WAIT (uint32_t)50

#define MAGIC_PSU_INIT (uint8_t)0x01
#define MAGIC_PSU_POLL_SERIAL (uint8_t)0x41
#define MAGIC_PSU_SET_VOLTAGE (uint8_t)0x42
#define MAGIC_PSU_POLL_AC_STATUS (uint8_t)0x23
#define MAGIC_PSU_SET_OUTPUT (uint8_t)0x29
#define MAGIC_PSU_POLL_OUTPUT (uint8_t)0x24
#define MAGIC_PSU_POLL_AC_PARAMETERS (uint8_t)0x46

#define PSU_IGNORE_OFFLINE_CYCLES (uint8_t)10

