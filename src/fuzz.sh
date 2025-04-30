#!/bin/fish

conda activate opencood
echo "Starting fuzzing process..."
echo "loading opt.py"
rm src/log/param/current_time.txt
python src/main.py
echo "Fuzzing process completed."
