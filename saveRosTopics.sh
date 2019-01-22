gnome-terminal -x bash -c "sleep 1 && while true; do rostopic echo /world_state > worldState.txt; done;" &
gnome-terminal -x bash -c "sleep 1 && while true; do rostopic echo /robotcommands > robotCommands.txt; done;" &

