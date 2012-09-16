/* ----------------------------------------------------------------------------
 *         ATMEL Microcontroller Software Support 
 * ----------------------------------------------------------------------------
 * Copyright (c) 2008, Atmel Corporation
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Atmel's name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ----------------------------------------------------------------------------
 */

#ifndef FATFS_CONFIG_H
#define FATFS_CONFIG_H

//------------------------------------------------------------------------------
//      General Definitions (previously in ff.h)
//------------------------------------------------------------------------------

#define	_FATFS_TINY 0
/* When _FATFS_TINY is set to 1, fatfs is compiled in Tiny mode
/ Else, it is compiled in normal mode 
/ Tiny FatFs feature : Very low memory consumption, suitable for small memory 
/ system. (1KB RAM) : Supports only single drive, no disk format, 
/ only read functions, no write functions */

//------------------------------------------------------------------------------
//      Definitions for normal FATFS (previously in ff.h)
//------------------------------------------------------------------------------

#if _FATFS_TINY == 0

#define _MCU_ENDIAN		2
/* The _MCU_ENDIAN defines which access method is used to the FAT structure.
/  1: Enable word access.
/  2: Disable word access and use byte-by-byte access instead.
/  When the architectural byte order of the MCU is big-endian and/or address
/  miss-aligned access results incorrect behavior, the _MCU_ENDIAN must be set to 2.
/  If it is not the case, it can also be set to 1 for good code efficiency. */

#define _FS_READONLY	0
/* Setting _FS_READONLY to 1 defines read only configuration. This removes
/  writing functions, f_write, f_sync, f_unlink, f_mkdir, f_chmod, f_rename,
/  f_truncate and useless f_getfree. */

#define _FS_MINIMIZE	0
/* The _FS_MINIMIZE option defines minimization level to remove some functions.
/  0: Full function.
/  1: f_stat, f_getfree, f_unlink, f_mkdir, f_chmod, f_truncate and f_rename are removed.
/  2: f_opendir and f_readdir are removed in addition to level 1.
/  3: f_lseek is removed in addition to level 2. */

#define	_USE_STRFUNC	0
/* To enable string functions, set _USE_STRFUNC to 1 or 2. */

#define _USE_FSINFO	1
/* To enable FSInfo support on FAT32 volume, set _USE_FSINFO to 1. */

#define	_USE_SJIS	1
/* When _USE_SJIS is set to 1, Shift-JIS code transparency is enabled, otherwise
/  only US-ASCII(7bit) code can be accepted as file/directory name. */

#define	_USE_NTFLAG	1
/* When _USE_NTFLAG is set to 1, upper/lower case of the file name is preserved.
/  Note that the files are always accessed in case insensitive. */

#define	_USE_MKFS	1
/* When _USE_MKFS is set to 1 and _FS_READONLY is set to 0, f_mkfs function is
/  enabled. */

#define _DRIVES		2
/* Number of logical drives to be used. This affects the size of internal table. */

#define	_MULTI_PARTITION	0
/* When _MULTI_PARTITION is set to 0, each logical drive is bound to same
/  physical drive number and can mount only 1st primaly partition. When it is
/  set to 1, each logical drive can mount a partition listed in Drives[]. */

//------------------------------------------------------------------------------
//      Definitions for normal FATFS TINY (previously in tff.h)
//------------------------------------------------------------------------------

#else

#define _MCU_ENDIAN		2
/* The _MCU_ENDIAN defines which access method is used to the FAT structure.
/  1: Enable word access.
/  2: Disable word access and use byte-by-byte access instead.
/  When the architectural byte order of the MCU is big-endian and/or address
/  miss-aligned access results incorrect behavior, the _MCU_ENDIAN must be set to 2.
/  If it is not the case, it can also be set to 1 for good code efficiency. */

#define _FS_READONLY	1
/* Setting _FS_READONLY to 1 defines read only configuration. This removes
/  writing functions, f_write, f_sync, f_unlink, f_mkdir, f_chmod, f_rename,
/  f_truncate, f_getfree and internal writing codes. */

#define _FS_MINIMIZE	0
/* The _FS_MINIMIZE option defines minimization level to remove some functions.
/  0: Full function.
/  1: f_stat, f_getfree, f_unlink, f_mkdir, f_chmod, f_truncate and f_rename are removed.
/  2: f_opendir and f_readdir are removed in addition to level 1.
/  3: f_lseek is removed in addition to level 2. */

#define	_USE_STRFUNC	0
/* To enable string functions, set _USE_STRFUNC to 1 or 2. */

#define _USE_FSINFO	1
/* To enable FSInfo support on FAT32 volume, set _USE_FSINFO to 1. */

#define	_USE_SJIS	1
/* When _USE_SJIS is set to 1, Shift-JIS code transparency is enabled, otherwise
/  only US-ASCII(7bit) code can be accepted as file/directory name. */

#define	_USE_NTFLAG	1
/* When _USE_NTFLAG is set to 1, upper/lower case of the file name is preserved.
/  Note that the files are always accessed in case insensitive. */

#define	_USE_FORWARD	0
/* To enable f_forward function, set _USE_FORWARD to 1. */

#define _FAT32	1
/* To enable FAT32 support in addition of FAT12/16, set _FAT32 to 1. */

#endif

//------------------------------------------------------------------------------
//      Other definitions
//------------------------------------------------------------------------------

/*-----------------------------------------------------------------------*/
/* Correspondence between drive number and physical drive                */
/* Note that Tiny-FatFs supports only single drive and always            */
/* accesses drive number 0.                                              */

#define DRV_MMC          0
#define DRV_NAND         1
#define DRV_SDRAM        2
#define DRV_ATA          3
#define DRV_USB          4

#define SECTOR_SIZE_DEFAULT 512

#endif // FATFS_CONFIG_H
