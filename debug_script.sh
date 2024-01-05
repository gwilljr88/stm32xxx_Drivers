#!/usr/bin/env zsh
echo "Running debug script to start Openocd and gdbfrontend. . . "
startOpenocd="openocd -f /usr/local/share/openocd/scripts/board/stm32f4discovery.cfg"
startgdbfrontend="~/Embedded/gdb-frontend/gdbfrontend -g \$(realpath ~/Embedded/gdb-13.2-build/gdb/gdb) -G --data-directory=\$(realpath ~/Embedded/gdb-13.2-build/gdb/data-directory)"
osascript -e "tell application \"Terminal\" to do script \"${startOpenocd}\""
osascript -e "tell application \"Terminal\" to do script \"${startgdbfrontend}\""
sleep 3
open -na "Google Chrome" --args --new-window http://127.0.0.1:5550/