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

//------------------------------------------------------------------------------
//         Headers
//------------------------------------------------------------------------------

#include <stdio.h>
#include <string.h>
#include "ff_util.h"

//------------------------------------------------------------------------------
//         Global functions
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/// Scan the directory passed in argument and display all the files and 
/// directories contained in it and in its subdirectories
/// (recursive function)
/// \param path  directory path to scan
//------------------------------------------------------------------------------
void FF_ScanDir(char* path)
{
    FILINFO finfo;
    DIR dirs;
    int i;

    if (f_opendir(&dirs, path) == FR_OK) {
      
        i = strlen(path);
        while ((f_readdir(&dirs, &finfo) == FR_OK) && finfo.fname[0]) {
          
            if (finfo.fattrib & AM_DIR) {
              
                sprintf(&path[i], "/%s", &finfo.fname[0]);
                printf("%s\n\r", path);
                FF_ScanDir(path);
                path[i] = 0;
            } 
            else {
                printf("%s/%s\n\r", path, &finfo.fname[0]);
            }
        }
    }
}

//------------------------------------------------------------------------------
/// Get Time for FatFs
//------------------------------------------------------------------------------
DWORD get_fattime( void )
{
    unsigned int date;

    date = ((2008 - 1980) << 25)
        | (01 << 21)
        | (14 << 16)
        | (WORD) (17 << 11)
        | (WORD) (16 << 5)
        | (WORD) (11 >> 1);

    return date;
}

//------------------------------------------------------------------------------
// Convert file access error number in string
//------------------------------------------------------------------------------
const char* FF_GetStrResult(FRESULT res)
{
    switch(res) {
        case FR_OK :              return "FR_OK";
        case FR_NOT_READY :       return "FR_NOT_READY";
        case FR_NO_FILE :         return "FR_NO_FILE";
        case FR_NO_PATH :         return "FR_NO_PATH";
        case FR_INVALID_NAME :    return "FR_INVALID_NAME";
        case FR_INVALID_DRIVE :   return "FR_INVALID_DRIVE";
        case FR_DENIED :          return "FR_DENIED";   
        case FR_EXIST :           return "FR_EXIST"; 
        case FR_RW_ERROR :        return "FR_RW_ERROR";
        case FR_WRITE_PROTECTED : return "FR_WRITE_PROTECTED";
        case FR_NOT_ENABLED :     return "FR_NOT_ENABLED";
        case FR_NO_FILESYSTEM :   return "FR_NO_FILESYSTEM";
        case FR_INVALID_OBJECT :  return "FR_INVALID_OBJECT";
        case FR_MKFS_ABORTED :    return "FR_MKFS_ABORTED";  
        default : return "?";
    }  
}
