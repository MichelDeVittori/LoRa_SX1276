#ifndef SETTINGS_H
#define	SETTINGS_H

/* Defines utility ************************************************************/
#define FALSE 0
#define TRUE  1

#define SCG     0
#define MSCG    1
#define NODE    2

/* Defines for Firmware *******************************************************/
#define LORA_MSG_ON
#define NL "\r\n"

/* Select if is Gateway Or Node and respective setting*/
#define BOARD_TYPE              NODE //NODE, SCG or MSCG
#define BOARD_MSCG_SCG_ADDRESS  1 
#define BOARD_NODE_ADDRESS      4

/* Other **********************************************************************/

#endif

