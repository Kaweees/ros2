```zsh
ros2 run demo_nodes_cpp listener
ros2 run demo_nodes_cpp talker
ros2_graph /talker /listener -o ./src/lab1/chatter_diagram.md
```

```zsh
rm -rf build/ install/ log/ && clear && colcon build --symlink-install --packages-select lab1 && source install/setup.zsh && ros2 launch lab1 listener_squared_launch.py
ros2_graph /talker /listener /squared -o ./src/lab1/listener_squared_diagram.md
```

```sh
ros2_ws/src/lab1/
└── src/
   ├── __init__.py
   ├── talker.py
   ├── listener.py
   └── squared.py
```

```mermaid
flowchart LR
/talker[ /talker ]:::main
/listener[ /listener ]:::main
/squared[ /squared ]:::main
/listener[ /listener ]:::node
/squared[ /squared ]:::node
/integer_topic([ /integer_topic<br>std_msgs/msg/Int32 ]):::topic
/squared_topic([ /squared_topic<br>std_msgs/msg/Int32 ]):::bugged
/integer_topic --> /listener
/integer_topic --> /squared
/integer_topic --> /listener
/integer_topic --> /squared
/talker --> /integer_topic
/squared --> /squared_topic

subgraph keys[<b>Keys<b/>]
subgraph nodes[<b><b/>]
topicb((No connected)):::bugged
main_node[main]:::main
end
subgraph connection[<b><b/>]
node1[node1]:::node
node2[node2]:::node
node1 o-.-o|to server| service[/Service<br>service/Type\]:::service
service <-.->|to client| node2
node1 -->|publish| topic([Topic<br>topic/Type]):::topic
topic -->|subscribe| node2
node1 o==o|to server| action{{/Action<br>action/Type/}}:::action
action <==>|to client| node2
end
end
classDef node opacity:0.9,fill:#2A0,stroke:#391,stroke-width:4px,color:#fff
classDef action opacity:0.9,fill:#66A,stroke:#225,stroke-width:2px,color:#fff
classDef service opacity:0.9,fill:#3B8062,stroke:#3B6062,stroke-width:2px,color:#fff
classDef topic opacity:0.9,fill:#852,stroke:#CCC,stroke-width:2px,color:#fff
classDef main opacity:0.9,fill:#059,stroke:#09F,stroke-width:4px,color:#fff
classDef bugged opacity:0.9,fill:#933,stroke:#800,stroke-width:2px,color:#fff
style keys opacity:0.15,fill:#FFF
style nodes opacity:0.15,fill:#FFF
style connection opacity:0.15,fill:#FFF
```
