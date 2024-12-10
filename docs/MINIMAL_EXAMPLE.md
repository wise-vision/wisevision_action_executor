# Minimal example
This minimal example demonstrates how to use the wisevision_action_executor package to create subscriber for any topic and trigger action if some of conditions are fulfilled. It guides you through add, delete and modify actions.
## Prerequisites
- [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html) or later
- [Build and run](BUILD_AND_RUN.md)
## Local run:
Assuming that after building the package, you can source the workspace, and run the node:
```bash
cd ~/wisevision_action_executor_ws
source install/setup.bash
ros2 run automatic_action_execution automatic_action_service
```
## Using the wisevision_action_executor
The `wisevision_action_executor` provides services to add, delete, modify actions and show available actions.

Services:
- `/create_automatic_action` (lora_msgs/srv/AutomaticAction): Creates action for one choose topic.
- `/create_combined_automatic_action` (lora_msgs/srv/AutomaticActionConnection): Creates combined action.
- `/delete_automatic_action` (lora_msgs/srv/AutomaticActionDelete): Deletes action for one choose action_name.
- `/delete_combined_automatic_action` (lora_msgs/srv/AutomaticActionCombinedDelete): Deletes combined action for one choose combined action name.
- `/change_automatic_action` (lora_msgs/srv/ChangeAutomaticAction): Changes parameters in action.
- `/change_combined_automatic_action` (lora_msgs/srv/ChangeAutomaticActionCombined): Changes parameters in combined action.
- `/available_topics` (lora_msgs/srv/AvailableTopics): Shows available actions.
- `/available_topics_combined` (lora_msgs/srv/AvailableTopicsCombined): Shows available combined actions.

Nested Messages:
- To get message nessted in array, whre e.g. has some private id, pass to `value` this varibale name like `array_name[id_name=X].nested_value`.
- To get message nessted in array, but without private id, but from  corrdinate in array pass to `value` this varibale name like `array_name[0]` and if it simple type like Float32 don't pass `array_name[0].data`.

## Examples
### Create single action
To add action, call the `/create_automatic_action`
``` bash
ros2 service call /create_automatic_action lora_msgs/srv/AutomaticAction "{listen_topic: '/topic_1', listen_message_type: 'std_msgs/msg/Int32', value: 'data', trigger_val: '50.0', trigger_type: 'LessThan', action_and_publisher_name: '/example_topic_1', pub_message_type: 'std_msgs/msg/String', trigger_text: 'test', data_validity_ms: 5000}"
```
### Create combined action for many single actions with logic expresion
To add action for many single actions with logic expresion, call the `/create_combined_automatic_action`:
```bash
ros2 service call /create_combined_automatic_action lora_msgs/srv/AutomaticActionConnection "{listen_topics: ['/topic_1', '/topic_2'], logic_expression: '/example_topic_1 and /example_topic_2', pub_topic: 'example_topic'}"
```
### Delete action
To delete action, call the `/delete_automatic_action`
``` bash
ros2 service call /delete_automatic_action lora_msgs/srv/AutomaticActionDelete "{listen_topic_to_delete: '/topic_1'}"
```

### Delete combinded action:
To delete combined action, call the `/delete_combined_automatic_action`:
```bash
ros2 service call /delete_combined_automatic_action lora_msgs/srv/AutomaticActionCombinedDelete "{name_of_combined_topics_publisher: 'topic_1_and_topic_2'}"
```

###  Change action:
To change parameters in action, call the `/change_automatic_action`:
```bash
ros2 service call /change_automatic_action lora_msgs/srv/ChangeAutomaticAction "{action_and_publisher_name_to_change: '/example_topic_1', listen_topic: '/topic_1', listen_message_type: 'std_msgs/msg/Int32', value: 'data', trigger_val: '75.0', trigger_type: 'LessThan', new_action_and_publisher_name: '/example_topic_3', pub_message_type: 'std_msgs/msg/String', trigger_text: 'test', data_validity_ms: 5000, publication_method: 0}"
```
### Change action combined:
To change parameters in combined action, call the `/change_combined_automatic_action`:
```bash
ros2 service call /change_combined_automatic_action lora_msgs/srv/ChangeAutomaticActionCombined "{
  action_and_publisher_name_to_change: 'example_topic_test',
  listen_topics: ['/example_topic_1', '/example_topic_2'],
  logic_expression: '/example_topic_1 and /example_topic_2',
  new_action_and_publisher_name: 'example_topic_test',
  trigger_text: 'Updated action triggered!',
  publication_method: 2
}"
```

### Check available actions:
To check available actions before create combined action, call the `/available_topics`:
```bash
ros2 service call /available_topics lora_msgs/srv/AvailableTopics 
```

### Available actions combined:
To check available combined actions, call the `/available_topics_combined`:
```bash
ros2 service call /available_topics_combined lora_msgs/srv/AvailableTopicsCombined
```
