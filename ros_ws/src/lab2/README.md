```zsh
rm -rf build/ install/ log/ && clear && colcon build --symlink-install --packages-select lab2 && source install/setup.zsh && ros2 launch lab2 draw_square_launch.py
```
