# Turtle Nest Tests

## How to run

1. Open the docker container (from the repository root)

    ```
    cd docker
    docker compose up -d
    docker exec -it turtle_nest bash
    ```

1. run:
   ```
   cd /home/user/ros2_ws/src/turtle_nest_tests/build/
   make
   ./turtle_nest_tests
   ```
   
Optionally, you can filter tests with:

```
./turtle_nest_tests --gtest_filter=<TestSuiteName>.<TestName>
```