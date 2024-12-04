/*!
    \file    readme.txt
    \brief   description of the CMP Window demo

    \version 2023-12-31, V2.3.0, firmware for GD32F3x0
*/

/*
    Copyright (c) 2023, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software without
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/

  This demo is based on the GD32350R-EVAL-V1.0 board, it shows how to make an analog 
watchdog by using comparator peripherals in window mode.

  CMP is configured as following:
  - Non Inverting input is connected to PA1.
  - The upper threshold(CMP0 inverting input) is set to VREFINT = 1.2V.
  - The lower threshold(CMP1 inverting input) is set to VREFINT / 2 = 0.6V.
  - The input voltage is configured to be connected to VR(PC1).

  After system start-up, enable comparator, the status of LEDs depend on the input voltage.
  If the input voltage is above the higher threshold, LED1,LED2 and LED3 are all turned on.
  If the input voltage is within the thresholds, LED1 and LED2 are turned on.
  If the input voltage is under the lower threshold, LED1 is turned on.
