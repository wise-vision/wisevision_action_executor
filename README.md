# wisevision_action_executor
This code provides functions to create subscriber for any topic and trigger action if some of conditions are fulfilled. The subscribtions can be removed. Program is persistence across restarts. PEX provides function for creating combined topics publisher.

## Table of contents
For more information, please refer to the following sections:

[Build and run(Start here)](docs/BUILD_AND_RUN.md)

[Minimal example](docs/MINIMAL_EXAMPLE.md)

[Interface design](docs/INTERFACE_DESIGN.md)

If you want to contribute to this project, please read the following sections:

[Code od conduct](docs/CODE_OF_CONDUCT.md)




## API
### Services

- /create_automatic_action
- /create_combined_automatic_action
- /delete_automatic_action
- /delete_combined_automatic_action
- /change_automatic_action
- /change_combined_automatic_action
- /available_topics
- /available_topics_combined

### Publishers
- /notifications
- /<publisher_created_by_user_in_service>