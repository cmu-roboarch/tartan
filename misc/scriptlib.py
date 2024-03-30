# MIT License
#

import re


DONT_CARE_FILES = [".*zsim/.*",
                   ".*apps/include/parallel_hashmap.*",
                   ".*apps/include/stb_image.h",
                   ".*apps/include/nlohmann.*",
                   ".*apps/include/vcl.*",
                   ".*apps/include/args.h",
                   ".*apps/include/log.h",
                   ".*apps/include/zsim_hooks.h",
                   ".*apps/HomeBot/glob.hpp",
                   ".*opencv_build/.*",
                   ".*pin-2.14-71313-gcc.4.4.7-linux/.*"
                   ]


def isDontCareFile(filePath):
    for dcf in DONT_CARE_FILES:
        m = re.search(dcf, str(filePath))
        if m: return True
    return False
