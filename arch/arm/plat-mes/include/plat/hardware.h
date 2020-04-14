#ifndef __LF1000_PLAT_HARDWARE_H__
#define __LF1000_PLAT_HARDWARE_H__

#ifdef CONFIG_ARCH_LF1000
#define cpu_is_lf1000()	(read_cpuid_id() == 0x41069265)
#else
#define cpu_is_lf1000()	(0)
#endif

#endif /* __LF1000_PLAT_HARDWARE_H__ */
