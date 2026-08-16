#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Fix the .uvprojx FilePath entries to be project-relative (prepend ..\)
   and switch the FreeRTOS port from GCC to RVDS (ARMCC V5 compatible)."""
import io, sys

UV = r"C:\Users\123\Desktop\合并项目\demo_seial_OLED_stm32f103rct6\Project\stm32f103RCT6.uvprojx"

with io.open(UV, "r", encoding="utf-8") as f:
    s = f.read()

# 1) prepend "..\" to FilePath entries that are root-relative for the
#    groups we added (Core / Startup / Middlewares / User). FWLib already
#    has "..\" so it is untouched.
for grp in ("Core", "Startup", "Middlewares", "User"):
    old = '<FilePath>%s\\' % grp
    new = '<FilePath>..\\%s\\' % grp
    s = s.replace(old, new)

# 2) switch FreeRTOS port GCC -> RVDS (covers FilePath and IncludePath)
s = s.replace('portable\\GCC\\ARM_CM3', 'portable\\RVDS\\ARM_CM3')

with io.open(UV, "w", encoding="utf-8") as f:
    f.write(s)

# report
import re
fp_broken = re.findall(r'<FilePath>(?:Core|Startup|Middlewares|User)\\', s)
fp_ok = re.findall(r'<FilePath>\.\.\\(?:Core|Startup|Middlewares|User)\\', s)
gcc = s.count('portable\\GCC\\ARM_CM3')
rvds = s.count('portable\\RVDS\\ARM_CM3')
print("remaining root-relative (broken) FilePath:", len(fp_broken))
print("project-relative (ok) FilePath:", len(fp_ok))
print("GCC port refs:", gcc, " RVDS port refs:", rvds)
