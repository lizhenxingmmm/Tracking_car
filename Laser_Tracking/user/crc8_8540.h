/**
 * @Author: Li Zhenxing
 * @Date: 2024/6/10 01:27:59
 * @LastEditors: Li Zhenxing
 * @LastEditTime: 2024/6/10 01:27:59
 * Description: 
 * Copyright: Copyright (©)}) 2024 Li Zhenxing. All rights reserved.
 */
#ifndef CRC8_8540_H
#define CRC8_8540_H

unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength, unsigned char ucCRC8);
unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);

#endif
