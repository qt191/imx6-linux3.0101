/* modify by zhengsh 2017-04-19
*  If only one machine board file (for example: only CONFIG_MACH_MX6Q_QIYANG = y ) is enabled,
*  the machine ID is also forced from uboot
*/
#define machine_arch_type __machine_arch_type 
#include <generated/mach-types.h>
