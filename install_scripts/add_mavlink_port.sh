#!/bin/bash

FICHIER="/home/tancrede_s/SWARMz4/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink"

#paragraph add
read -r -d '' PARAGRAPHE <<'EOF'

udp_param_port_local=$((14590+px4_instance))
udp_param_port_remote=$((14560+px4_instance))
# Custom MAVLink link for monitoring/tuning
mavlink start -x -u $udp_param_port_local -o $udp_param_port_remote -r 4000000 -f -m onboard

EOF

# Verified if already insert
if grep -q "udp_param_port_local" "$FICHIER"; then
  echo "Paragraphe déjà présent, aucune modification."
else
  tmpfile=$(mktemp)
  awk -v paragraph="$PARAGRAPHE" '
    /# GCS link/ {
      print paragraph;
      print;
      next
    }
    { print }
  ' "$FICHIER" > "$tmpfile"
  mv "$tmpfile" "$FICHIER"
  echo "Paragraphe inséré après la ligne '# GCS link'"
fi
