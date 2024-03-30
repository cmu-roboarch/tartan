# MIT License

DONT_CARE_FILES=(
    "*zsim/*"
    "*apps/include/parallel_hashmap*"
    "*apps/include/stb_image.h"
    "*apps/include/nlohmann*"
    "*apps/include/vcl*"
    "*apps/include/args.h"
    "*apps/include/log.h"
    "*apps/include/zsim_hooks.h"
    "*apps/HomeBot/glob.hpp"
    "*opencv_build/*"
    "*pin-2.14-71313-gcc.4.4.7-linux/*"
    )


is_dont_care_file() {
    local FILE_PATH=$1
    IS_DCF=0
    for DCF in "${DONT_CARE_FILES[@]}"; do
        if [[ $FILE_PATH == $DCF ]]; then
            IS_DCF=1
        fi
    done

    echo "$IS_DCF"
}
