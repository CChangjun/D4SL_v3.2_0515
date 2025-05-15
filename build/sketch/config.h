#line 1 "C:\\개인폴더\\FirmWare\\무선화FW\\TEST\\CHANG__cust_0508\\JUST_TEST\\D4SLv3.1\\CHANG_cust_d4sl\\config.h"
/*
 * 
 */
#define TEST_RESPONSE_DELAY       0


#define FUNC_WL_REPAIRING_DELAY   1
#define FUNC_WL_ALL_RELEASE       1

#define SYS_SETUP_WL_GROUP        1

typedef struct _sys_config {

  unsigned int group_WL;   // WL group
  unsigned int channel_WL;   // WL channel
  unsigned int addr_WL;   // WL address
} sys_config; 

